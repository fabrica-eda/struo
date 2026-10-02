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
