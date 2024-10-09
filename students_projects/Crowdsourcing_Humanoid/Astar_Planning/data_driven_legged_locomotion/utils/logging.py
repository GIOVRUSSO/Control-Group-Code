import logging
import csv
import io

class CsvFormatter(logging.Formatter):
    def __init__(self):
        super().__init__()
        self.output = io.StringIO()
        self.writer = csv.writer(self.output, quoting=csv.QUOTE_ALL)

    def format(self, record):
        if isinstance(record.msg, list) or isinstance(record.msg, tuple):
            self.writer.writerow(record.msg)
        else:
            self.writer.writerow([record.msg])
        data = self.output.getvalue()
        self.output.truncate(0)
        self.output.seek(0)
        return data.strip()

logging.basicConfig(level=logging.DEBUG)
