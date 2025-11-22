class Test:
    def __init__(self):
        self.flag=True
        self.message=""

    @property
    def Warp(self,fun):
        def Tmp():
            if self.flag:
                fun()
            else:
                print("else")
                self.message="false"
        return Tmp

    @Warp
    def Operate(self):
        print("operate")


if __name__ == '__main__':
    a=Test()

    a.Operate()