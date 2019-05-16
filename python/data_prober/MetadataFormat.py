
class MetadataFormat:

    def __init__(self):
        self.dataFields = []

    def __repr__(self):
        return "MetadataFormat()"

    def __str__(self):
        res = str(len(self.dataFields)) + " data fields :\n"
        count = 0
        for name in self.dataFields:
            res += str(count) + " : " + name + "\n"
            count += 1
        return res

    def parse_metadata_format_file(self, filename):
 
        formatFile = open(filename, "r")
        for line in formatFile:
            self.dataFields.append(line.split()[3])
    
    def write_metadata_format_file(self, filename):
        formatFile = open(filename, "w")
        index = 1
        for field in self.dataFields:
            if len(self.dataFields) >= 100:
                line = "# " + format(index, '>3d') + ' - ' + field
            else:
                line = "# " + format(index, '>2d') + ' - ' + field
            formatFile.write(line + "\n")
            index = index + 1
