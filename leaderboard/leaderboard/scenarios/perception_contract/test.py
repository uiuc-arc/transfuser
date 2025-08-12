from z3 import *

x = Real('x')

def foo():
    x1 = Real('x')
    return (x1 ** 3 > 10)

exp = foo()
s = Solver()
s.add(exp)
res = s.check()
if res == sat:
    model = s.model()
    print("SAT")
    for d in model.decls():
        print(f"{d.name()} = {model[d]}")
else:
    print("UNSAT")

s.add(x < 0)
res = s.check()
if res == sat:
    model = s.model()
    print("SAT")
    for d in model.decls():
        print(f"{d.name()} = {model[d]}")
else:
    print("UNSAT")
    print(s.unsat_core())