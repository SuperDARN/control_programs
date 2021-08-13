from config.StationConfig import Experiment

from T3_Object import T3_Object


class Normalscan16(Experiment):
    def __init__(self):
        super().__init__()
        self.name = "Normalscan16"
        self.cp = 151
        self.scanbound = 60000  # ms
        self.mpinc = 2700  # us
        self.nrang = 210  # range gates
        # self.frang = 180  # first range gate, in km
        self.beam_order = range(18, 2, -1)

        self.frequency = 13.5e6
