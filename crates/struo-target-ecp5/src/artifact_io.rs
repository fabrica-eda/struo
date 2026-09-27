//! Versioned, compressed binary artifacts with legacy JSON read compatibility.
use serde::{Serialize, de::DeserializeOwned};
use std::error::Error;
use std::fs::File;
use std::io::{BufRead, BufReader, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"STRUOA\x01\n";

/// Read a binary artifact, or a legacy JSON artifact.
///
/// # Errors
/// Rejects malformed, truncated or unsupported input.
pub fn read_artifact<T: DeserializeOwned>(path: &Path) -> Result<T, Box<dyn Error>> {
    let mut input = BufReader::new(File::open(path)?);
    if input.fill_buf()?.starts_with(MAGIC) {
        input.consume(MAGIC.len());
        let mut decoder = zstd::stream::read::Decoder::new(input)?;
        let value = ciborium::de::from_reader(&mut decoder)?;
        // Consume the frame footer too: decoding the CBOR value alone does not
        // validate a truncated frame or its checksum.
        let mut trailing = [0];
        if decoder.read(&mut trailing)? != 0 {
            return Err("trailing data in binary artifact".into());
        }
        Ok(value)
    } else {
        Ok(serde_json::from_reader(input)?)
    }
}

/// Atomically replace a artifact with CBOR inside a checksummed Zstd frame.
/// No expanded JSON representation is written or constructed by this writer.
///
/// # Errors
/// Returns serialization, compression or filesystem errors. Before replacement,
/// a failed write leaves the previous artifact intact.
pub fn write_artifact_binary<T: Serialize>(path: &Path, value: &T) -> Result<(), Box<dyn Error>> {
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent)?;
    let mut temporary = tempfile::NamedTempFile::new_in(parent)?;
    temporary.write_all(MAGIC)?;
    {
        let mut encoder = zstd::stream::write::Encoder::new(temporary.as_file_mut(), 3)?;
        encoder.include_checksum(true)?;
        ciborium::ser::into_writer(value, &mut encoder)?;
        encoder.finish()?;
    }
    temporary.as_file().sync_all()?;
    temporary.persist(path)?;
    #[cfg(unix)]
    File::open(parent)?.sync_all()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::ser::{SerializeMap, Serializer};
    use serde_json::{Value, json};

    #[test]
    fn binary_and_legacy_preserve_unsigned_ids_and_timing() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("nested/design.stnet");
        let value = json!({"wire":u64::MAX,"slack":-2796,"hold":-36,"routes":[{"pips":[0,1,2]}]});
        write_artifact_binary(&path, &value).unwrap();
        assert!(std::fs::read(&path).unwrap().starts_with(MAGIC));
        assert_eq!(read_artifact::<Value>(&path).unwrap(), value);
        let old = directory.path().join("legacy.json");
        std::fs::write(&old, serde_json::to_vec(&value).unwrap()).unwrap();
        assert_eq!(read_artifact::<Value>(&old).unwrap(), value);
    }

    #[test]
    fn failed_serialization_preserves_existing_artifact() {
        struct Fails;
        impl Serialize for Fails {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                let mut map = serializer.serialize_map(Some(2))?;
                map.serialize_entry("partial", &42)?;
                Err(serde::ser::Error::custom("injected failure"))
            }
        }
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("design.stnet");
        write_artifact_binary(&path, &json!({"old":true})).unwrap();
        let before = std::fs::read(&path).unwrap();
        assert!(write_artifact_binary(&path, &Fails).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), before);
        assert_eq!(std::fs::read_dir(directory.path()).unwrap().count(), 1);
    }

    #[test]
    fn truncated_footer_and_corruption_are_rejected() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("design.stnet");
        write_artifact_binary(&path, &json!({"data":vec![123;1000]})).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        for length in [0, 7, 8, bytes.len() - 1, bytes.len() - 4] {
            std::fs::write(&path, &bytes[..length]).unwrap();
            assert!(read_artifact::<Value>(&path).is_err());
        }
        let mut corrupted = bytes;
        let last = corrupted.len() - 1;
        corrupted[last] ^= 1;
        std::fs::write(&path, corrupted).unwrap();
        assert!(read_artifact::<Value>(&path).is_err());
    }
}
