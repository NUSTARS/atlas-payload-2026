def calculate_crc(data):
    """Calculates the 16-bit CRC checksum for the given byte sequence."""
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | (crc << 8)) & 0xffff
        crc ^= byte
        crc ^= ((crc & 0xff) >> 4)
        crc ^= (crc << 12) & 0xffff
        crc ^= ((crc & 0x00ff) << 5) & 0xffff
    return crc & 0xffff


def check_vn_message(vn_message_hex):
    """Validates a VN message by checking its checksum."""
    if len(vn_message_hex) < 4:
        print("invalid")
        return
    
    # Extract the message data and checksum (last 4 hex chars = 2 bytes)
    message_hex = vn_message_hex[:-4]
    provided_checksum_hex = vn_message_hex[-4:]
    
    # Convert provided checksum from hex string to integer (little-endian)
    try:
        # VectorNav uses little-endian byte order for checksum
        provided_checksum = int(provided_checksum_hex[0:2] + provided_checksum_hex[2:4], 16)
    except ValueError:
        print("invalid")
        return
    
    # Convert message hex string to bytes
    try:
        data_bytes = bytes.fromhex(message_hex)
    except ValueError:
        print("invalid")
        return
    
    # Calculate CRC
    calculated_checksum = calculate_crc(data_bytes)
    
    # Compare checksums
    if calculated_checksum == provided_checksum:
        print("valid")
    else:
        print(f"invalid - calculated: {calculated_checksum:04X}, provided: {provided_checksum:04X}")


# Example usage
if __name__ == "__main__":
    # VN message string with checksum at the end (last 2 hex digits)
    vn_message = "FA1291"
    check_vn_message(vn_message)
