class MotorStatusParser:
    def parse_line(self, line):
        if line.startswith("Position:"):
            try:
                parts = line.split(',')
                pos = parts[0].split(':')[1].strip()
                vel = parts[1].split(':')[1].strip()
                tor = parts[2].split(':')[1].strip()
                return {"pos": pos, "vel": vel, "tor": tor}
            except:
                return None
        return None
