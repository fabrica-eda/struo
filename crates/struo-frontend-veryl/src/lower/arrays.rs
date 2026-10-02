use veryl_analyzer::ir::ArrayLiteralItem;

use super::{ImportError, constant_value};

// Plan placement before evaluating expressions so a default can be evaluated
// at its source position even though it fills elements after explicit items.
pub(super) fn literal_repetitions(
    items: &[ArrayLiteralItem],
    length: usize,
) -> Result<Vec<usize>, ImportError> {
    let mut repetitions = Vec::new();
    let mut explicit = 0usize;
    let mut default = None;
    for (index, item) in items.iter().enumerate() {
        let count = match item {
            ArrayLiteralItem::Value(_, repeat) => {
                let count = repeat
                    .as_ref()
                    .map(|repeat| constant_value(repeat))
                    .transpose()?
                    .unwrap_or(1);
                let count = usize::try_from(count).map_err(|_| {
                    ImportError::UnsupportedBehavior("array repetition overflow".into())
                })?;
                if count > length.saturating_sub(explicit) {
                    return Err(ImportError::UnsupportedBehavior(
                        "array literal shape mismatch".into(),
                    ));
                }
                explicit += count;
                count
            }
            ArrayLiteralItem::Defaul(_) => {
                if default.replace(index).is_some() {
                    return Err(ImportError::UnsupportedBehavior(
                        "multiple array literal defaults".into(),
                    ));
                }
                0
            }
        };
        repetitions.push(count);
    }
    if let Some(index) = default {
        repetitions[index] = length - explicit;
    } else if explicit != length {
        return Err(ImportError::UnsupportedBehavior(
            "incomplete array literal".into(),
        ));
    }
    Ok(repetitions)
}

impl super::ModuleLowerer<'_> {
    pub(super) fn is_constant_variable(&self, id: super::VarId) -> bool {
        self.source.variables.get(&id).is_some_and(|variable| {
            matches!(variable.kind, super::VarKind::Const | super::VarKind::Param)
        })
    }

    pub(super) fn lower_constant_variable_read(
        &mut self,
        id: super::VarId,
        index: &super::VarIndex,
        env: &mut super::Env,
        effects: &mut super::DrivenBits,
    ) -> Result<super::LoweredExpr, ImportError> {
        if !super::has_dynamic_array_index(index) {
            let key = self.key_from_index(id, index)?;
            return self.lower_constant_array_element(&key);
        }
        let keys = super::array_indices(self.variable_type(id)?, "constant array")?
            .into_iter()
            .map(|index| super::SignalKey { id, index })
            .collect();
        let candidates = self.lower_array_candidates_effects(id, index, keys, env, effects)?;
        let mut entries = Vec::with_capacity(candidates.len());
        for (key, condition) in candidates {
            entries.push((condition, self.lower_constant_array_element(&key)?));
        }
        // Reuse the balanced selector; the unmatched result follows the
        // mapped two-state backend's existing unpacked-array read policy.
        let (matched, value) = self.lower_array_read_tree(&entries)?;
        let zero = self.constant(value.width, 0);
        Ok(super::LoweredExpr {
            id: self.rtl.mux(matched.id, value.id, zero.id)?,
            ..value
        })
    }

    fn lower_constant_array_element(
        &mut self,
        key: &super::SignalKey,
    ) -> Result<super::LoweredExpr, ImportError> {
        let variable = &self.source.variables[&key.id];
        let value = variable.get_value(&key.index).ok_or_else(|| {
            ImportError::UnsupportedBehavior(format!(
                "constant array element {} has no numeric value",
                self.signal_name(key)
            ))
        })?;
        if value.is_xz() {
            return Err(ImportError::UnsupportedBehavior(
                "unknown or four-state constant array element".into(),
            ));
        }
        let width = super::concrete_width(&variable.r#type, "constant array element")?;
        let signed = variable.r#type.signed;
        let value = value.expand(width as usize, signed);
        Ok(super::LoweredExpr {
            id: self.rtl.constant(super::Constant::new(
                super::BitWidth::new(width)?,
                value.payload().to_u64_digits(),
            )),
            width,
            signed,
        })
    }
}
