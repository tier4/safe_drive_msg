/// Currently, just skip macros.
pub fn preprocess(input: &str) -> String {
    let mut result = String::new();

    for line in input.split('\n') {
        let line = line.trim();
        if !line.starts_with("#") {
            result.push_str(&format!("{line}\n"));
        }
    }

    result
}
