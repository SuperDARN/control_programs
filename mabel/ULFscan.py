from config.StationConfig import Experiment

from T3_Object import T3_Object

class ULFscan(Experiment):
    def __init__(self):
        super().__init__()
        self.name = 'ULFscan'
        self.cp = 8020
        self.scanbound = 60000  # ms
        self.mpinc = 2700  # us
        self.nrang = 210  # range gates
        # self.frang = 180  # first range gate, in km
        self.beam_order = [12, 10, 8, 4, 12, 10, 8, 4, 12, 10, 8, 4, 12, 10, 8, 4]
        self.frequency = 13.5e6
