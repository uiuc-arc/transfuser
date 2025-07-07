from z3 import *

def add_segment(solver, param, p_start, p_end, var, eqn):
    """Add an equation segment to the solver."""
    segment = Implies(And(param >= p_start, param < p_end), 
                      var == eqn[0] + eqn[1] * param + eqn[2] * param * param)
    solver.add(segment)


    