# How do classes work?

class Status():
    pass

def main():
    Status.a = 1;
    def sub():
        print(Status.a)
    sub()

main()
