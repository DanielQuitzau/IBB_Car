# IBB_Car/utils/message_parser.py

def parse_arduino_message(line):
    """
    Erwartet Nachrichten wie:
    - D:<distanz_cm>
    - IR:<wert>
    Gibt ein Dict zurück, z.B. {"distance": 123}
    """
    if not line or ":" not in line:
        return None

    key, value = line.split(":", 1)
    try:
        value = float(value)
    except ValueError:
        pass

    if key == "D":
        return {"distance": value}
    elif key == "IR":
        return {"ir": value}
    return None
