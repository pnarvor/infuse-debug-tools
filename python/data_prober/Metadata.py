import matplotlib.pyplot as plt
import numpy as np
import os
from .MetadataFormat import MetadataFormat

class Metadata:


    def __init__(self):
        self.metadataFormat = MetadataFormat()

    def __repr__(self):
        return "Metadata()"

    def __str__(self):
        return self.metadataFormat.__str__()

    def __getitem__(self, key):
        return getattr(self, key)

    def parse_metadata(self, metadataFormatFilename, metadataFilename):

        if not os.path.isfile(metadataFormatFilename):
            raise Exception("Metadata could not find file : ", metadataFormatFilename)
        if  not os.path.isfile(metadataFilename):
            raise Exception("Metadata could not find file : ", metadataFilename)

        self.metadataFormat.parse_metadata_format_file(metadataFormatFilename)

        setattr(self, "index", [])
        for name in self.metadataFormat.dataFields:
            setattr(self, name, [])
       
        metadataFile = open(metadataFilename, "r")
    
        nbAttributes = len(self.__dict__.keys())
        indexCount = 0
        for line in metadataFile:
            self.index.append(indexCount)
            words = line.split()
            if len(words) != nbAttributes - 2:
                raise Exception("Error : Number of columns (" + str(len(words))
                                + ") isn't equal to the number of metadata fields descripted in "
                                + metadataFilename + " (" + str(nbAttributes - 2) + ")")
            count = 0
            for word in words:
                try:
                    getattr(self, self.metadataFormat.dataFields[count]).append(float(word))
                except ValueError:
                    getattr(self, self.metadataFormat.dataFields[count]).append(word)
                count += 1
            indexCount += 1

    def get(self, field):
        if type(field) is int:
            return getattr(self, self.metadataFormat.dataFields[field])
        elif type(field) is str:
            return getattr(self, field)
        else:
            raise Exception("get(field) : field must be a field name or an integer")

    def get_nparray(self, field):
        if type(field) is int:
            return np.array(getattr(self, self.metadataFormat.dataFields[field]))
        elif type(field) is str:
            return np.array(getattr(self, field))
        else:
            raise Exception("get(field) : field must be a field name or an integer")

    def add_field(self, name):
        
        if hasattr(self, name):
            raise Exception("Metadata has already attribute \"" + name + "\" !")
        self.metadataFormat.append(name)
        setattr(self, name);
        self[name] = []

    def write_metadata_files(self, path):
        
        dataFormatFilename  = os.path.join(path, "dataformat.txt")
        allMetadataFilename = os.path.join(path, "all_metadata.txt")
        metadataFolder      = os.path.join(path, "metadata")

        # Creating metadata files folder
        if (os.path.exists(dataFormatFilename)
        or  os.path.exists(allMetadataFilename)
        or  os.path.exists(metadataFolder)):
            raise Exception("Metadata already written in path !")
        
        # creating the dataformat file
        self.metadataFormat.write_metadata_format_file(dataFormatFilename)

        # creating data dir (one file per line)
        os.mkdir(metadataFolder)

        # building a list of list of data value
        dataToWrite = []
        for index in self['index']:
            dataToWrite.append([index])
        for field in self.metadataFormat.dataFields:
            if field == 'index':
                continue
            for dataLine, value in zip(dataToWrite, self[field]):
                dataLine.append(value)

        allMetadataFile = open(allMetadataFilename, 'w')
        for dataLine in dataToWrite:
            filename = format(dataLine[0], '05d') + ".txt"
            metadataFile = open(os.path.join(metadataFolder, filename), 'w')
            
            # Building data string
            dataString = str(dataLine[1])
            for value in dataLine[2:]:
                dataString = dataString + " " + str(value)
            
            metadataFile.write(dataString)
            allMetadataFile.write(dataString + "\n")
            metadataFile.close()
        
    def plot(self, field):
        plt.plot(self.get(field))
        plt.show()

