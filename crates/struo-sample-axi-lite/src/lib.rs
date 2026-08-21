//! Protocol-oriented synthesis stress designs.
//!
//! The crossbar in this crate is a fixed 2-by-2 AXI4-Lite fabric. It keeps the
//! five AXI channels independent, buffers write addresses and data separately,
//! applies backpressure, returns `DECERR` for unmapped addresses, and uses
//! round-robin arbitration when both initiators request the same target.

use struo_rtl::{
    BinaryOp, BitWidth, ClockEdge, Constant, Design, ExprId, Module, Polarity, Port, PortDirection,
    Register, Reset, ResetMode, RtlError, SignalId, StateDomain, UnaryOp, ValueType,
};

const ADDRESS_WIDTH: u32 = 16;
const DATA_WIDTH: u32 = 32;
const STROBE_WIDTH: u32 = DATA_WIDTH / 8;
const PROTECTION_WIDTH: u32 = 3;
const RESPONSE_WIDTH: u32 = 2;
const DECERR: u64 = 0b11;

/// Builds a two-initiator, two-target AXI4-Lite crossbar.
///
/// Target zero owns `0x0000..=0x3fff`; target one owns
/// `0x8000..=0xbfff`. Other addresses complete locally with `DECERR`.
/// Each initiator and target may have one write and one read outstanding.
///
/// # Errors
///
/// Returns an error if construction violates the frontend-independent RTL
/// invariants.
pub fn axi_lite_crossbar_2x2() -> Result<Design, RtlError> {
    let mut module = Module::new("AxiLiteCrossbar2x2");
    let clock = add_input(&mut module, "clk", 1);
    let reset_n = add_input(&mut module, "rst_n", 1);
    let initiators = (0..2)
        .map(|index| add_initiator_ports(&mut module, index))
        .collect::<Vec<_>>();
    let targets = (0..2)
        .map(|index| add_target_ports(&mut module, index))
        .collect::<Vec<_>>();

    build_write_path(&mut module, clock, reset_n, &initiators, &targets)?;
    build_read_path(&mut module, clock, reset_n, &initiators, &targets)?;
    module.validate()?;

    let mut design = Design::new("AxiLiteCrossbar2x2");
    design.add_module(module);
    Ok(design)
}

#[derive(Clone, Copy)]
struct InitiatorPorts {
    awaddr: SignalId,
    awprot: SignalId,
    awvalid: SignalId,
    awready: SignalId,
    wdata: SignalId,
    wstrb: SignalId,
    wvalid: SignalId,
    wready: SignalId,
    bresp: SignalId,
    bvalid: SignalId,
    bready: SignalId,
    araddr: SignalId,
    arprot: SignalId,
    arvalid: SignalId,
    arready: SignalId,
    rdata: SignalId,
    rresp: SignalId,
    rvalid: SignalId,
    rready: SignalId,
}

#[derive(Clone, Copy)]
struct TargetPorts {
    awaddr: SignalId,
    awprot: SignalId,
    awvalid: SignalId,
    awready: SignalId,
    wdata: SignalId,
    wstrb: SignalId,
    wvalid: SignalId,
    wready: SignalId,
    bresp: SignalId,
    bvalid: SignalId,
    bready: SignalId,
    araddr: SignalId,
    arprot: SignalId,
    arvalid: SignalId,
    arready: SignalId,
    rdata: SignalId,
    rresp: SignalId,
    rvalid: SignalId,
    rready: SignalId,
}

struct State {
    name: String,
    signal: SignalId,
    value: ExprId,
    width: u32,
}

struct WriteIngress {
    aw_full: State,
    awaddr: State,
    awprot: State,
    w_full: State,
    wdata: State,
    wstrb: State,
    busy: State,
    error_valid: State,
}

struct WriteTarget {
    active: State,
    owner: State,
    aw_sent: State,
    w_sent: State,
    awaddr: State,
    awprot: State,
    wdata: State,
    wstrb: State,
    round_robin: State,
}

struct ReadIngress {
    busy: State,
    error_valid: State,
}

struct ReadTarget {
    active: State,
    owner: State,
    ar_sent: State,
    araddr: State,
    arprot: State,
    round_robin: State,
}

#[derive(Clone, Copy)]
struct WindowDecode {
    target: [ExprId; 2],
    error: ExprId,
}

fn add_initiator_ports(module: &mut Module, index: usize) -> InitiatorPorts {
    let prefix = format!("s{index}");
    InitiatorPorts {
        awaddr: add_input(module, &format!("{prefix}_awaddr"), ADDRESS_WIDTH),
        awprot: add_input(module, &format!("{prefix}_awprot"), PROTECTION_WIDTH),
        awvalid: add_input(module, &format!("{prefix}_awvalid"), 1),
        awready: add_output(module, &format!("{prefix}_awready"), 1),
        wdata: add_input(module, &format!("{prefix}_wdata"), DATA_WIDTH),
        wstrb: add_input(module, &format!("{prefix}_wstrb"), STROBE_WIDTH),
        wvalid: add_input(module, &format!("{prefix}_wvalid"), 1),
        wready: add_output(module, &format!("{prefix}_wready"), 1),
        bresp: add_output(module, &format!("{prefix}_bresp"), RESPONSE_WIDTH),
        bvalid: add_output(module, &format!("{prefix}_bvalid"), 1),
        bready: add_input(module, &format!("{prefix}_bready"), 1),
        araddr: add_input(module, &format!("{prefix}_araddr"), ADDRESS_WIDTH),
        arprot: add_input(module, &format!("{prefix}_arprot"), PROTECTION_WIDTH),
        arvalid: add_input(module, &format!("{prefix}_arvalid"), 1),
        arready: add_output(module, &format!("{prefix}_arready"), 1),
        rdata: add_output(module, &format!("{prefix}_rdata"), DATA_WIDTH),
        rresp: add_output(module, &format!("{prefix}_rresp"), RESPONSE_WIDTH),
        rvalid: add_output(module, &format!("{prefix}_rvalid"), 1),
        rready: add_input(module, &format!("{prefix}_rready"), 1),
    }
}

fn add_target_ports(module: &mut Module, index: usize) -> TargetPorts {
    let prefix = format!("m{index}");
    TargetPorts {
        awaddr: add_output(module, &format!("{prefix}_awaddr"), ADDRESS_WIDTH),
        awprot: add_output(module, &format!("{prefix}_awprot"), PROTECTION_WIDTH),
        awvalid: add_output(module, &format!("{prefix}_awvalid"), 1),
        awready: add_input(module, &format!("{prefix}_awready"), 1),
        wdata: add_output(module, &format!("{prefix}_wdata"), DATA_WIDTH),
        wstrb: add_output(module, &format!("{prefix}_wstrb"), STROBE_WIDTH),
        wvalid: add_output(module, &format!("{prefix}_wvalid"), 1),
        wready: add_input(module, &format!("{prefix}_wready"), 1),
        bresp: add_input(module, &format!("{prefix}_bresp"), RESPONSE_WIDTH),
        bvalid: add_input(module, &format!("{prefix}_bvalid"), 1),
        bready: add_output(module, &format!("{prefix}_bready"), 1),
        araddr: add_output(module, &format!("{prefix}_araddr"), ADDRESS_WIDTH),
        arprot: add_output(module, &format!("{prefix}_arprot"), PROTECTION_WIDTH),
        arvalid: add_output(module, &format!("{prefix}_arvalid"), 1),
        arready: add_input(module, &format!("{prefix}_arready"), 1),
        rdata: add_input(module, &format!("{prefix}_rdata"), DATA_WIDTH),
        rresp: add_input(module, &format!("{prefix}_rresp"), RESPONSE_WIDTH),
        rvalid: add_input(module, &format!("{prefix}_rvalid"), 1),
        rready: add_output(module, &format!("{prefix}_rready"), 1),
    }
}

fn add_input(module: &mut Module, name: &str, width: u32) -> SignalId {
    module.add_port(Port {
        name: name.into(),
        direction: PortDirection::Input,
        r#type: value_type(width),
    })
}

fn add_output(module: &mut Module, name: &str, width: u32) -> SignalId {
    module.add_port(Port {
        name: name.into(),
        direction: PortDirection::Output,
        r#type: value_type(width),
    })
}

fn value_type(width: u32) -> ValueType {
    ValueType {
        width: BitWidth::new(width).expect("AXI widths are non-zero"),
        signed: false,
        state: StateDomain::TwoState,
    }
}

impl State {
    fn new(module: &mut Module, name: impl Into<String>, width: u32) -> Result<Self, RtlError> {
        let name = name.into();
        let signal = module.add_signal(&name, value_type(width));
        let value = module.read(signal)?;
        Ok(Self {
            name,
            signal,
            value,
            width,
        })
    }
}

impl WriteIngress {
    fn new(module: &mut Module, index: usize) -> Result<Self, RtlError> {
        let prefix = format!("wr_s{index}");
        Ok(Self {
            aw_full: State::new(module, format!("{prefix}_aw_full"), 1)?,
            awaddr: State::new(module, format!("{prefix}_awaddr"), ADDRESS_WIDTH)?,
            awprot: State::new(module, format!("{prefix}_awprot"), PROTECTION_WIDTH)?,
            w_full: State::new(module, format!("{prefix}_w_full"), 1)?,
            wdata: State::new(module, format!("{prefix}_wdata"), DATA_WIDTH)?,
            wstrb: State::new(module, format!("{prefix}_wstrb"), STROBE_WIDTH)?,
            busy: State::new(module, format!("{prefix}_busy"), 1)?,
            error_valid: State::new(module, format!("{prefix}_error_valid"), 1)?,
        })
    }
}

impl WriteTarget {
    fn new(module: &mut Module, index: usize) -> Result<Self, RtlError> {
        let prefix = format!("wr_m{index}");
        Ok(Self {
            active: State::new(module, format!("{prefix}_active"), 1)?,
            owner: State::new(module, format!("{prefix}_owner"), 1)?,
            aw_sent: State::new(module, format!("{prefix}_aw_sent"), 1)?,
            w_sent: State::new(module, format!("{prefix}_w_sent"), 1)?,
            awaddr: State::new(module, format!("{prefix}_awaddr"), ADDRESS_WIDTH)?,
            awprot: State::new(module, format!("{prefix}_awprot"), PROTECTION_WIDTH)?,
            wdata: State::new(module, format!("{prefix}_wdata"), DATA_WIDTH)?,
            wstrb: State::new(module, format!("{prefix}_wstrb"), STROBE_WIDTH)?,
            round_robin: State::new(module, format!("{prefix}_round_robin"), 1)?,
        })
    }
}

impl ReadIngress {
    fn new(module: &mut Module, index: usize) -> Result<Self, RtlError> {
        let prefix = format!("rd_s{index}");
        Ok(Self {
            busy: State::new(module, format!("{prefix}_busy"), 1)?,
            error_valid: State::new(module, format!("{prefix}_error_valid"), 1)?,
        })
    }
}

impl ReadTarget {
    fn new(module: &mut Module, index: usize) -> Result<Self, RtlError> {
        let prefix = format!("rd_m{index}");
        Ok(Self {
            active: State::new(module, format!("{prefix}_active"), 1)?,
            owner: State::new(module, format!("{prefix}_owner"), 1)?,
            ar_sent: State::new(module, format!("{prefix}_ar_sent"), 1)?,
            araddr: State::new(module, format!("{prefix}_araddr"), ADDRESS_WIDTH)?,
            arprot: State::new(module, format!("{prefix}_arprot"), PROTECTION_WIDTH)?,
            round_robin: State::new(module, format!("{prefix}_round_robin"), 1)?,
        })
    }
}

#[allow(
    clippy::needless_range_loop,
    clippy::similar_names,
    clippy::too_many_lines
)]
fn build_write_path(
    module: &mut Module,
    clock: SignalId,
    reset_n: SignalId,
    ports: &[InitiatorPorts],
    target_ports: &[TargetPorts],
) -> Result<(), RtlError> {
    let ingress = (0..2)
        .map(|index| WriteIngress::new(module, index))
        .collect::<Result<Vec<_>, _>>()?;
    let targets = (0..2)
        .map(|index| WriteTarget::new(module, index))
        .collect::<Result<Vec<_>, _>>()?;
    let awvalid = read_ports(module, ports, |port| port.awvalid)?;
    let awaddr = read_ports(module, ports, |port| port.awaddr)?;
    let awprot = read_ports(module, ports, |port| port.awprot)?;
    let wvalid = read_ports(module, ports, |port| port.wvalid)?;
    let wdata = read_ports(module, ports, |port| port.wdata)?;
    let wstrb = read_ports(module, ports, |port| port.wstrb)?;
    let bready = read_ports(module, ports, |port| port.bready)?;
    let target_awready = read_target_ports(module, target_ports, |port| port.awready)?;
    let target_wready = read_target_ports(module, target_ports, |port| port.wready)?;
    let target_bvalid = read_target_ports(module, target_ports, |port| port.bvalid)?;
    let target_bresp = read_target_ports(module, target_ports, |port| port.bresp)?;
    let decode = ingress
        .iter()
        .map(|state| decode_address(module, state.awaddr.value))
        .collect::<Result<Vec<_>, _>>()?;

    let mut grants = Vec::with_capacity(2);
    for target in 0..2 {
        let eligible_0 = write_eligible(module, &ingress[0], decode[0].target[target])?;
        let eligible_1 = write_eligible(module, &ingress[1], decode[1].target[target])?;
        grants.push(round_robin_grant(
            module,
            eligible_0,
            eligible_1,
            targets[target].round_robin.value,
            targets[target].active.value,
        )?);
    }

    let mut response_valid = Vec::with_capacity(2);
    let mut target_done = Vec::with_capacity(2);
    for target in 0..2 {
        let state = &targets[target];
        let awvalid_out = and_not(module, state.active.value, state.aw_sent.value)?;
        let wvalid_out = and_not(module, state.active.value, state.w_sent.value)?;
        assign(module, target_ports[target].awvalid, awvalid_out)?;
        assign(module, target_ports[target].wvalid, wvalid_out)?;
        assign(module, target_ports[target].awaddr, state.awaddr.value)?;
        assign(module, target_ports[target].awprot, state.awprot.value)?;
        assign(module, target_ports[target].wdata, state.wdata.value)?;
        assign(module, target_ports[target].wstrb, state.wstrb.value)?;

        let channels_sent = and(module, state.aw_sent.value, state.w_sent.value)?;
        let response_phase = and(module, state.active.value, channels_sent)?;
        let owner_ready = mux(module, state.owner.value, bready[1], bready[0])?;
        let bready_out = and(module, response_phase, owner_ready)?;
        assign(module, target_ports[target].bready, bready_out)?;
        let response = and(module, response_phase, target_bvalid[target])?;
        let owner_is_0 = not(module, state.owner.value)?;
        response_valid.push([
            and(module, response, owner_is_0)?,
            and(module, response, state.owner.value)?,
        ]);
        target_done.push(and(module, response, owner_ready)?);
    }

    let mut dispatch = Vec::with_capacity(2);
    let mut error_grant = Vec::with_capacity(2);
    let mut response_handshake = Vec::with_capacity(2);
    for initiator in 0..2 {
        let both_full = and(
            module,
            ingress[initiator].aw_full.value,
            ingress[initiator].w_full.value,
        )?;
        let not_busy = not(module, ingress[initiator].busy.value)?;
        let error_ready = and(module, both_full, not_busy)?;
        let error = and(module, error_ready, decode[initiator].error)?;
        error_grant.push(error);
        let target_dispatch = or(module, grants[0][initiator], grants[1][initiator])?;
        dispatch.push(or(module, target_dispatch, error)?);

        let forwarded_valid = or(
            module,
            response_valid[0][initiator],
            response_valid[1][initiator],
        )?;
        let bvalid_out = or(
            module,
            ingress[initiator].error_valid.value,
            forwarded_valid,
        )?;
        let zero_response = constant(module, RESPONSE_WIDTH, 0);
        let response_0 = mux(
            module,
            response_valid[0][initiator],
            target_bresp[0],
            zero_response,
        )?;
        let forwarded_response = mux(
            module,
            response_valid[1][initiator],
            target_bresp[1],
            response_0,
        )?;
        let decode_error = constant(module, RESPONSE_WIDTH, DECERR);
        let bresp_out = mux(
            module,
            ingress[initiator].error_valid.value,
            decode_error,
            forwarded_response,
        )?;
        assign(module, ports[initiator].bvalid, bvalid_out)?;
        assign(module, ports[initiator].bresp, bresp_out)?;
        response_handshake.push(and(module, bvalid_out, bready[initiator])?);
    }

    for initiator in 0..2 {
        let state = &ingress[initiator];
        let not_aw_full = not(module, state.aw_full.value)?;
        let not_busy = not(module, state.busy.value)?;
        let awready_out = and(module, not_aw_full, not_busy)?;
        let not_w_full = not(module, state.w_full.value)?;
        let wready_out = and(module, not_w_full, not_busy)?;
        assign(module, ports[initiator].awready, awready_out)?;
        assign(module, ports[initiator].wready, wready_out)?;
        let accept_aw = and(module, awvalid[initiator], awready_out)?;
        let accept_w = and(module, wvalid[initiator], wready_out)?;

        let aw_full_next = flag_next(module, state.aw_full.value, accept_aw, dispatch[initiator])?;
        let awaddr_next = capture_next(module, state.awaddr.value, accept_aw, awaddr[initiator])?;
        let awprot_next = capture_next(module, state.awprot.value, accept_aw, awprot[initiator])?;
        let w_full_next = flag_next(module, state.w_full.value, accept_w, dispatch[initiator])?;
        let wdata_next = capture_next(module, state.wdata.value, accept_w, wdata[initiator])?;
        let wstrb_next = capture_next(module, state.wstrb.value, accept_w, wstrb[initiator])?;
        let busy_next = flag_next(
            module,
            state.busy.value,
            dispatch[initiator],
            response_handshake[initiator],
        )?;
        let error_valid_next = flag_next(
            module,
            state.error_valid.value,
            error_grant[initiator],
            response_handshake[initiator],
        )?;
        add_state_register(module, &state.aw_full, aw_full_next, clock, reset_n)?;
        add_state_register(module, &state.awaddr, awaddr_next, clock, reset_n)?;
        add_state_register(module, &state.awprot, awprot_next, clock, reset_n)?;
        add_state_register(module, &state.w_full, w_full_next, clock, reset_n)?;
        add_state_register(module, &state.wdata, wdata_next, clock, reset_n)?;
        add_state_register(module, &state.wstrb, wstrb_next, clock, reset_n)?;
        add_state_register(module, &state.busy, busy_next, clock, reset_n)?;
        add_state_register(module, &state.error_valid, error_valid_next, clock, reset_n)?;
    }

    for target in 0..2 {
        let state = &targets[target];
        let grant_any = or(module, grants[target][0], grants[target][1])?;
        let active_next = flag_next(module, state.active.value, grant_any, target_done[target])?;
        let owner_next = capture_next(module, state.owner.value, grant_any, grants[target][1])?;
        let aw_not_sent = not(module, state.aw_sent.value)?;
        let aw_handshake = and3(
            module,
            state.active.value,
            aw_not_sent,
            target_awready[target],
        )?;
        let w_not_sent = not(module, state.w_sent.value)?;
        let w_handshake = and3(
            module,
            state.active.value,
            w_not_sent,
            target_wready[target],
        )?;
        let channel_clear = or(module, grant_any, target_done[target])?;
        let aw_sent_next = flag_next(module, state.aw_sent.value, aw_handshake, channel_clear)?;
        let w_sent_next = flag_next(module, state.w_sent.value, w_handshake, channel_clear)?;
        let captured_awaddr = mux(
            module,
            grants[target][1],
            ingress[1].awaddr.value,
            ingress[0].awaddr.value,
        )?;
        let captured_awprot = mux(
            module,
            grants[target][1],
            ingress[1].awprot.value,
            ingress[0].awprot.value,
        )?;
        let captured_wdata = mux(
            module,
            grants[target][1],
            ingress[1].wdata.value,
            ingress[0].wdata.value,
        )?;
        let captured_wstrb = mux(
            module,
            grants[target][1],
            ingress[1].wstrb.value,
            ingress[0].wstrb.value,
        )?;
        let awaddr_next = capture_next(module, state.awaddr.value, grant_any, captured_awaddr)?;
        let awprot_next = capture_next(module, state.awprot.value, grant_any, captured_awprot)?;
        let wdata_next = capture_next(module, state.wdata.value, grant_any, captured_wdata)?;
        let wstrb_next = capture_next(module, state.wstrb.value, grant_any, captured_wstrb)?;
        let next_preference = not(module, state.owner.value)?;
        let round_robin_next = capture_next(
            module,
            state.round_robin.value,
            target_done[target],
            next_preference,
        )?;

        add_state_register(module, &state.active, active_next, clock, reset_n)?;
        add_state_register(module, &state.owner, owner_next, clock, reset_n)?;
        add_state_register(module, &state.aw_sent, aw_sent_next, clock, reset_n)?;
        add_state_register(module, &state.w_sent, w_sent_next, clock, reset_n)?;
        add_state_register(module, &state.awaddr, awaddr_next, clock, reset_n)?;
        add_state_register(module, &state.awprot, awprot_next, clock, reset_n)?;
        add_state_register(module, &state.wdata, wdata_next, clock, reset_n)?;
        add_state_register(module, &state.wstrb, wstrb_next, clock, reset_n)?;
        add_state_register(module, &state.round_robin, round_robin_next, clock, reset_n)?;
    }
    Ok(())
}

#[allow(clippy::needless_range_loop, clippy::too_many_lines)]
fn build_read_path(
    module: &mut Module,
    clock: SignalId,
    reset_n: SignalId,
    ports: &[InitiatorPorts],
    target_ports: &[TargetPorts],
) -> Result<(), RtlError> {
    let ingress = (0..2)
        .map(|index| ReadIngress::new(module, index))
        .collect::<Result<Vec<_>, _>>()?;
    let targets = (0..2)
        .map(|index| ReadTarget::new(module, index))
        .collect::<Result<Vec<_>, _>>()?;
    let araddr = read_ports(module, ports, |port| port.araddr)?;
    let arprot = read_ports(module, ports, |port| port.arprot)?;
    let arvalid = read_ports(module, ports, |port| port.arvalid)?;
    let rready = read_ports(module, ports, |port| port.rready)?;
    let target_arready = read_target_ports(module, target_ports, |port| port.arready)?;
    let target_rdata = read_target_ports(module, target_ports, |port| port.rdata)?;
    let target_rresp = read_target_ports(module, target_ports, |port| port.rresp)?;
    let target_rvalid = read_target_ports(module, target_ports, |port| port.rvalid)?;
    let decode = araddr
        .iter()
        .map(|address| decode_address(module, *address))
        .collect::<Result<Vec<_>, _>>()?;

    let mut grants = Vec::with_capacity(2);
    for target in 0..2 {
        let eligible_0 = read_eligible(
            module,
            arvalid[0],
            ingress[0].busy.value,
            decode[0].target[target],
        )?;
        let eligible_1 = read_eligible(
            module,
            arvalid[1],
            ingress[1].busy.value,
            decode[1].target[target],
        )?;
        grants.push(round_robin_grant(
            module,
            eligible_0,
            eligible_1,
            targets[target].round_robin.value,
            targets[target].active.value,
        )?);
    }

    let mut response_valid = Vec::with_capacity(2);
    let mut target_done = Vec::with_capacity(2);
    for target in 0..2 {
        let state = &targets[target];
        let arvalid_out = and_not(module, state.active.value, state.ar_sent.value)?;
        assign(module, target_ports[target].arvalid, arvalid_out)?;
        assign(module, target_ports[target].araddr, state.araddr.value)?;
        assign(module, target_ports[target].arprot, state.arprot.value)?;
        let response_phase = and(module, state.active.value, state.ar_sent.value)?;
        let owner_ready = mux(module, state.owner.value, rready[1], rready[0])?;
        let rready_out = and(module, response_phase, owner_ready)?;
        assign(module, target_ports[target].rready, rready_out)?;
        let response = and(module, response_phase, target_rvalid[target])?;
        let owner_is_0 = not(module, state.owner.value)?;
        response_valid.push([
            and(module, response, owner_is_0)?,
            and(module, response, state.owner.value)?,
        ]);
        target_done.push(and(module, response, owner_ready)?);
    }

    let mut accepted = Vec::with_capacity(2);
    let mut error_accept = Vec::with_capacity(2);
    let mut response_handshake = Vec::with_capacity(2);
    for initiator in 0..2 {
        let not_busy = not(module, ingress[initiator].busy.value)?;
        let error_ready = and(module, not_busy, decode[initiator].error)?;
        let error = and(module, arvalid[initiator], error_ready)?;
        error_accept.push(error);
        let target_accept = or(module, grants[0][initiator], grants[1][initiator])?;
        accepted.push(or(module, target_accept, error)?);
        let arready_out = or(module, target_accept, error)?;
        assign(module, ports[initiator].arready, arready_out)?;

        let forwarded_valid = or(
            module,
            response_valid[0][initiator],
            response_valid[1][initiator],
        )?;
        let rvalid_out = or(
            module,
            ingress[initiator].error_valid.value,
            forwarded_valid,
        )?;
        let zero_data = constant(module, DATA_WIDTH, 0);
        let data_0 = mux(
            module,
            response_valid[0][initiator],
            target_rdata[0],
            zero_data,
        )?;
        let forwarded_data = mux(
            module,
            response_valid[1][initiator],
            target_rdata[1],
            data_0,
        )?;
        let zero_response = constant(module, RESPONSE_WIDTH, 0);
        let response_0 = mux(
            module,
            response_valid[0][initiator],
            target_rresp[0],
            zero_response,
        )?;
        let forwarded_response = mux(
            module,
            response_valid[1][initiator],
            target_rresp[1],
            response_0,
        )?;
        let decode_error = constant(module, RESPONSE_WIDTH, DECERR);
        let rresp_out = mux(
            module,
            ingress[initiator].error_valid.value,
            decode_error,
            forwarded_response,
        )?;
        assign(module, ports[initiator].rvalid, rvalid_out)?;
        assign(module, ports[initiator].rdata, forwarded_data)?;
        assign(module, ports[initiator].rresp, rresp_out)?;
        response_handshake.push(and(module, rvalid_out, rready[initiator])?);
    }

    for initiator in 0..2 {
        let state = &ingress[initiator];
        let busy_next = flag_next(
            module,
            state.busy.value,
            accepted[initiator],
            response_handshake[initiator],
        )?;
        let error_valid_next = flag_next(
            module,
            state.error_valid.value,
            error_accept[initiator],
            response_handshake[initiator],
        )?;
        add_state_register(module, &state.busy, busy_next, clock, reset_n)?;
        add_state_register(module, &state.error_valid, error_valid_next, clock, reset_n)?;
    }

    for target in 0..2 {
        let state = &targets[target];
        let grant_any = or(module, grants[target][0], grants[target][1])?;
        let active_next = flag_next(module, state.active.value, grant_any, target_done[target])?;
        let owner_next = capture_next(module, state.owner.value, grant_any, grants[target][1])?;
        let ar_not_sent = not(module, state.ar_sent.value)?;
        let ar_handshake = and3(
            module,
            state.active.value,
            ar_not_sent,
            target_arready[target],
        )?;
        let channel_clear = or(module, grant_any, target_done[target])?;
        let ar_sent_next = flag_next(module, state.ar_sent.value, ar_handshake, channel_clear)?;
        let captured_araddr = mux(module, grants[target][1], araddr[1], araddr[0])?;
        let captured_arprot = mux(module, grants[target][1], arprot[1], arprot[0])?;
        let araddr_next = capture_next(module, state.araddr.value, grant_any, captured_araddr)?;
        let arprot_next = capture_next(module, state.arprot.value, grant_any, captured_arprot)?;
        let next_preference = not(module, state.owner.value)?;
        let round_robin_next = capture_next(
            module,
            state.round_robin.value,
            target_done[target],
            next_preference,
        )?;

        add_state_register(module, &state.active, active_next, clock, reset_n)?;
        add_state_register(module, &state.owner, owner_next, clock, reset_n)?;
        add_state_register(module, &state.ar_sent, ar_sent_next, clock, reset_n)?;
        add_state_register(module, &state.araddr, araddr_next, clock, reset_n)?;
        add_state_register(module, &state.arprot, arprot_next, clock, reset_n)?;
        add_state_register(module, &state.round_robin, round_robin_next, clock, reset_n)?;
    }
    Ok(())
}

fn write_eligible(
    module: &mut Module,
    state: &WriteIngress,
    address_hit: ExprId,
) -> Result<ExprId, RtlError> {
    let buffers_full = and(module, state.aw_full.value, state.w_full.value)?;
    let not_busy = not(module, state.busy.value)?;
    and3(module, buffers_full, not_busy, address_hit)
}

fn read_eligible(
    module: &mut Module,
    valid: ExprId,
    busy: ExprId,
    address_hit: ExprId,
) -> Result<ExprId, RtlError> {
    let not_busy = not(module, busy)?;
    and3(module, valid, not_busy, address_hit)
}

fn round_robin_grant(
    module: &mut Module,
    eligible_0: ExprId,
    eligible_1: ExprId,
    prefer_1: ExprId,
    active: ExprId,
) -> Result<[ExprId; 2], RtlError> {
    let not_eligible_0 = not(module, eligible_0)?;
    let one_priority = or(module, not_eligible_0, prefer_1)?;
    let select_1 = and(module, eligible_1, one_priority)?;
    let not_select_1 = not(module, select_1)?;
    let select_0 = and(module, eligible_0, not_select_1)?;
    let available = not(module, active)?;
    Ok([
        and(module, available, select_0)?,
        and(module, available, select_1)?,
    ])
}

fn decode_address(module: &mut Module, address: ExprId) -> Result<WindowDecode, RtlError> {
    let limit_0 = constant(module, ADDRESS_WIDTH, 0x4000);
    let base_1 = constant(module, ADDRESS_WIDTH, 0x8000);
    let limit_1 = constant(module, ADDRESS_WIDTH, 0xc000);
    let target_0 = module.binary(BinaryOp::LessThanUnsigned, address, limit_0)?;
    let above_base_1 = module.binary(BinaryOp::GreaterOrEqualUnsigned, address, base_1)?;
    let below_limit_1 = module.binary(BinaryOp::LessThanUnsigned, address, limit_1)?;
    let target_1 = and(module, above_base_1, below_limit_1)?;
    let mapped = or(module, target_0, target_1)?;
    let error = not(module, mapped)?;
    Ok(WindowDecode {
        target: [target_0, target_1],
        error,
    })
}

fn read_ports(
    module: &mut Module,
    ports: &[InitiatorPorts],
    select: impl Fn(&InitiatorPorts) -> SignalId,
) -> Result<Vec<ExprId>, RtlError> {
    ports.iter().map(|port| module.read(select(port))).collect()
}

fn read_target_ports(
    module: &mut Module,
    ports: &[TargetPorts],
    select: impl Fn(&TargetPorts) -> SignalId,
) -> Result<Vec<ExprId>, RtlError> {
    ports.iter().map(|port| module.read(select(port))).collect()
}

fn assign(module: &mut Module, signal: SignalId, value: ExprId) -> Result<(), RtlError> {
    let target = module.whole(signal)?;
    module.assign(target, value)
}

fn add_state_register(
    module: &mut Module,
    state: &State,
    next: ExprId,
    clock: SignalId,
    reset_n: SignalId,
) -> Result<(), RtlError> {
    let reset_value = constant(module, state.width, 0);
    module.add_register(Register {
        name: state.name.clone(),
        target: state.signal,
        next,
        clock,
        edge: ClockEdge::Rising,
        enable: None,
        reset: Some(Reset {
            signal: reset_n,
            mode: ResetMode::Asynchronous,
            polarity: Polarity::ActiveLow,
            value: reset_value,
        }),
    })
}

fn flag_next(
    module: &mut Module,
    current: ExprId,
    set: ExprId,
    clear: ExprId,
) -> Result<ExprId, RtlError> {
    let zero = constant(module, 1, 0);
    let one = constant(module, 1, 1);
    let after_set = mux(module, set, one, current)?;
    mux(module, clear, zero, after_set)
}

fn capture_next(
    module: &mut Module,
    current: ExprId,
    capture: ExprId,
    value: ExprId,
) -> Result<ExprId, RtlError> {
    mux(module, capture, value, current)
}

fn constant(module: &mut Module, width: u32, value: u64) -> ExprId {
    module.constant(Constant::from_u64(
        BitWidth::new(width).expect("AXI widths are non-zero"),
        value,
    ))
}

fn not(module: &mut Module, value: ExprId) -> Result<ExprId, RtlError> {
    module.unary(UnaryOp::LogicNot, value)
}

fn and(module: &mut Module, lhs: ExprId, rhs: ExprId) -> Result<ExprId, RtlError> {
    module.binary(BinaryOp::And, lhs, rhs)
}

fn and_not(module: &mut Module, lhs: ExprId, rhs: ExprId) -> Result<ExprId, RtlError> {
    let rhs = not(module, rhs)?;
    and(module, lhs, rhs)
}

fn and3(
    module: &mut Module,
    first: ExprId,
    second: ExprId,
    third: ExprId,
) -> Result<ExprId, RtlError> {
    let first_two = and(module, first, second)?;
    and(module, first_two, third)
}

fn or(module: &mut Module, lhs: ExprId, rhs: ExprId) -> Result<ExprId, RtlError> {
    module.binary(BinaryOp::Or, lhs, rhs)
}

fn mux(
    module: &mut Module,
    condition: ExprId,
    then_value: ExprId,
    else_value: ExprId,
) -> Result<ExprId, RtlError> {
    module.mux(condition, then_value, else_value)
}

#[cfg(test)]
mod tests {
    use celox::{JitBackend, Simulator};
    use struo_celox::ecp5_simulator;
    use struo_synth::synthesize;
    use struo_target_ecp5::map_to_ecp5;

    use super::axi_lite_crossbar_2x2;

    #[test]
    fn synthesizes_and_maps_the_crossbar() {
        let design = axi_lite_crossbar_2x2().unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();

        assert_eq!(synthesized.netlist.registers().len(), 288);
        assert!(mapped.cells().len() > synthesized.netlist.registers().len());
        ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap();
    }

    #[test]
    fn preserves_axi_lite_channel_and_backpressure_semantics() {
        let mut simulator = crossbar_simulator();
        reset(&mut simulator);
        exercise_independent_write_channels(&mut simulator);
        exercise_decode_error(&mut simulator);
        exercise_read_backpressure(&mut simulator);
        exercise_round_robin_arbitration(&mut simulator);
    }

    fn crossbar_simulator() -> Simulator<JitBackend> {
        let design = axi_lite_crossbar_2x2().unwrap();
        let synthesized = synthesize(&design).unwrap();
        let mapped = map_to_ecp5(&synthesized.netlist).unwrap();
        ecp5_simulator(&mapped).unwrap().build_cranelift().unwrap()
    }

    fn reset(simulator: &mut Simulator<JitBackend>) {
        set_u8(simulator, "rst_n", 0);
        let clock = simulator.event("clk");
        simulator.tick(clock).unwrap();
        set_u8(simulator, "rst_n", 1);
    }

    fn exercise_independent_write_channels(simulator: &mut Simulator<JitBackend>) {
        let clock = simulator.event("clk");
        set_u16(simulator, "s0_awaddr", 0x0010);
        set_u8(simulator, "s0_awprot", 0b010);
        set_u8(simulator, "s0_awvalid", 1);
        assert_value(simulator, "s0_awready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_awvalid", 0);

        set_u32(simulator, "s0_wdata", 0x1234_5678);
        set_u8(simulator, "s0_wstrb", 0b1111);
        set_u8(simulator, "s0_wvalid", 1);
        assert_value(simulator, "s0_wready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_wvalid", 0);
        simulator.tick(clock).unwrap();

        assert_value(simulator, "m0_awvalid", 1);
        assert_value(simulator, "m0_awaddr", 0x0010);
        assert_value(simulator, "m0_awprot", 0b010);
        assert_value(simulator, "m0_wvalid", 1);
        assert_value(simulator, "m0_wdata", 0x1234_5678);
        assert_value(simulator, "m0_wstrb", 0b1111);
        set_u8(simulator, "m0_wready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_wready", 0);
        assert_value(simulator, "m0_wvalid", 0);
        assert_value(simulator, "m0_awvalid", 1);

        set_u8(simulator, "m0_awready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_awready", 0);
        set_u8(simulator, "m0_bvalid", 1);
        set_u8(simulator, "m0_bresp", 0b10);
        assert_value(simulator, "s0_bvalid", 1);
        assert_value(simulator, "s0_bresp", 0b10);
        assert_value(simulator, "m0_bready", 0);
        simulator.tick(clock).unwrap();
        assert_value(simulator, "s0_bvalid", 1);

        set_u8(simulator, "s0_bready", 1);
        assert_value(simulator, "m0_bready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_bready", 0);
        set_u8(simulator, "m0_bvalid", 0);
        assert_value(simulator, "s0_bvalid", 0);
        assert_value(simulator, "s0_awready", 1);
        assert_value(simulator, "s0_wready", 1);
    }

    fn exercise_decode_error(simulator: &mut Simulator<JitBackend>) {
        let clock = simulator.event("clk");
        set_u16(simulator, "s1_araddr", 0x5000);
        set_u8(simulator, "s1_arvalid", 1);
        assert_value(simulator, "s1_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s1_arvalid", 0);
        assert_value(simulator, "s1_rvalid", 1);
        assert_value(simulator, "s1_rresp", 0b11);
        assert_value(simulator, "s1_rdata", 0);
        simulator.tick(clock).unwrap();
        assert_value(simulator, "s1_rvalid", 1);

        set_u8(simulator, "s1_rready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s1_rready", 0);
        assert_value(simulator, "s1_rvalid", 0);
    }

    fn exercise_read_backpressure(simulator: &mut Simulator<JitBackend>) {
        let clock = simulator.event("clk");
        set_u16(simulator, "s0_araddr", 0x8004);
        set_u8(simulator, "s0_arprot", 0b101);
        set_u8(simulator, "s0_arvalid", 1);
        assert_value(simulator, "s0_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_arvalid", 0);
        assert_value(simulator, "m1_arvalid", 1);
        assert_value(simulator, "m1_araddr", 0x8004);
        assert_value(simulator, "m1_arprot", 0b101);
        simulator.tick(clock).unwrap();
        assert_value(simulator, "m1_arvalid", 1);

        set_u8(simulator, "m1_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m1_arready", 0);
        set_u32(simulator, "m1_rdata", 0xdead_beef);
        set_u8(simulator, "m1_rresp", 0);
        set_u8(simulator, "m1_rvalid", 1);
        assert_value(simulator, "s0_rvalid", 1);
        assert_value(simulator, "s0_rdata", 0xdead_beef);
        assert_value(simulator, "s0_rresp", 0);
        assert_value(simulator, "m1_rready", 0);
        simulator.tick(clock).unwrap();
        assert_value(simulator, "s0_rvalid", 1);

        set_u8(simulator, "s0_rready", 1);
        assert_value(simulator, "m1_rready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_rready", 0);
        set_u8(simulator, "m1_rvalid", 0);
        assert_value(simulator, "s0_rvalid", 0);
    }

    fn exercise_round_robin_arbitration(simulator: &mut Simulator<JitBackend>) {
        let clock = simulator.event("clk");
        set_u16(simulator, "s0_araddr", 0x0020);
        set_u16(simulator, "s1_araddr", 0x0024);
        set_u8(simulator, "s0_arvalid", 1);
        set_u8(simulator, "s1_arvalid", 1);
        assert_value(simulator, "s0_arready", 1);
        assert_value(simulator, "s1_arready", 0);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_arvalid", 0);
        assert_value(simulator, "m0_araddr", 0x0020);
        set_u8(simulator, "m0_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_arready", 0);
        set_u8(simulator, "m0_rvalid", 1);
        set_u8(simulator, "s0_rready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_rvalid", 0);
        set_u8(simulator, "s0_rready", 0);

        set_u8(simulator, "s0_arvalid", 1);
        assert_value(simulator, "s0_arready", 0);
        assert_value(simulator, "s1_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "s0_arvalid", 0);
        set_u8(simulator, "s1_arvalid", 0);
        assert_value(simulator, "m0_araddr", 0x0024);
        set_u8(simulator, "m0_arready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_arready", 0);
        set_u8(simulator, "m0_rvalid", 1);
        set_u8(simulator, "s1_rready", 1);
        simulator.tick(clock).unwrap();
        set_u8(simulator, "m0_rvalid", 0);
        set_u8(simulator, "s1_rready", 0);
    }

    fn set_u8(simulator: &mut Simulator<JitBackend>, name: &str, value: u8) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u16(simulator: &mut Simulator<JitBackend>, name: &str, value: u16) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn set_u32(simulator: &mut Simulator<JitBackend>, name: &str, value: u32) {
        let signal = simulator.signal(name);
        simulator.modify(|io| io.set(signal, value)).unwrap();
    }

    fn assert_value(simulator: &mut Simulator<JitBackend>, name: &str, expected: u64) {
        assert_eq!(
            simulator.get(simulator.signal(name)),
            expected.into(),
            "{name}"
        );
    }
}
