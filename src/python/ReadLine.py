import time

encoding = "utf-8"

# Helper class to return serial output on a list of custom delimiters
# Will return no matter what after timeout has passed.
class ReadLine:
    def __init__(self, serial, timeout=2, delimiters=["\n", "CCC"]):
        self.serial = serial
        self.timeout = timeout
        self.clear()
        self.set_delimiters(delimiters)

    def encode(self, data):
        return data.encode(encoding)

    def clear(self):
        self.buf = bytearray()

    def set_serial(self, serial):
        self.serial = serial

    def set_timeout(self, timeout):
        self.timeout = timeout

    def set_delimiters(self, delimiters):
        self.delimiters = [
            bytes(d, encoding) if type(d) == str else d for d in delimiters]

    def read_line(self, timeout=None):
        if timeout is None or timeout <= 0.:
            timeout = self.timeout
        buf = self.buf
        for delimiter in self.delimiters:
            i = buf.find(delimiter)
            if i >= 0:
                offset = i+len(delimiter)
                r = buf[:offset]
                self.buf = buf[offset:]
                return self.__convert_to_str(r)

        start = time.time()
        while ((time.time() - start) < timeout):
            i = max(0, min(2048, self.serial.in_waiting))
            data = self.serial.read(i)
            if not data:
                continue
            for delimiter in self.delimiters:
                i = bytearray(buf + data).find(delimiter)
                if i >= 0:
                    offset = i+len(delimiter)
                    r = buf + data[:offset]
                    self.buf = bytearray(data[offset:])
                    return self.__convert_to_str(r)
            # No match
            buf.extend(data)

        # Timeout, reset buffer and return empty string
        #print("TIMEOUT! Got:\n>>>>>>\n{}\n<<<<<<\n".format(buf))
        self.buf = bytearray()
        return ""

    def __convert_to_str(self, data):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            return ""
