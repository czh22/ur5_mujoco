# 均值滤波函数

class average_filter():
    def __init__(self,N):
        self.data_buffer = []
        self.index = 0
        self.N = N

        for i in range(N):
            self.data_buffer.append(0)
    def filter(self,data):
        if self.index == self.N:
            self.index = 0
        self.data_buffer[self.index] = data
        self.index = self.index + 1
        # print(self.data_buffer)
        return sum(self.data_buffer)/len(self.data_buffer)
