/// Check if an interface has carrier (stub - always returns true on macOS)
pub fn has_carrier(_interface_name: &str) -> bool {
    // TODO: Implement proper carrier detection on macOS
    true
}

pub fn is_wireless_interface(interface_name: &str) -> bool {
    use system_configuration::dynamic_store::*;

    let store = SCDynamicStoreBuilder::new("networkInterfaceInfo").build();

    let key = format!("State:/Network/Interface/{}/AirPort", interface_name);
    if let Some(_) = store.get(key.as_str()) {
        return true;
    }
    false
}
