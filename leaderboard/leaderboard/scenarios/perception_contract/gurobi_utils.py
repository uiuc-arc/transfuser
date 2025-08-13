
def get_full_constraint_details(model, constraint):
    """
    Get complete details of a constraint including all variables and coefficients
    
    Args:
        model: Gurobi model
        constraint: Gurobi constraint object
        
    Returns:
        dict: Complete constraint information
    """
    # Basic constraint info
    info = {
        'name': constraint.ConstrName,
        'sense': constraint.Sense,
        'rhs': constraint.RHS,
        'slack': constraint.Slack if hasattr(constraint, 'Slack') else None,
        'dual': constraint.Pi if hasattr(constraint, 'Pi') else None,
        'terms': [],
        'constant': 0.0
    }
    
    # Get the linear expression (row)
    row = model.getRow(constraint)
    
    # Extract all terms from the linear expression
    for i in range(row.size()):
        var = row.getVar(i)
        coeff = row.getCoeff(i)
        info['terms'].append({
            'variable': var.VarName,
            'coefficient': coeff,
            'var_value': var.X if hasattr(var, 'X') else None,  # Current solution value
            'var_lb': var.LB,
            'var_ub': var.UB,
            'var_type': var.VType
        })
    
    # Get constant term if any
    info['constant'] = row.getConstant()
    
    return info

def export_constraints_to_file(model, filename="constraints_detailed.txt"):
    """Export all constraint details to a text file"""
    with open(filename, 'w') as f:
        f.write("GUROBI MODEL CONSTRAINT DETAILS\n")
        f.write("="*50 + "\n\n")
        
        constraints = model.getConstrs()
        for i, constr in enumerate(constraints):
            details = get_full_constraint_details(model, constr)
            
            f.write(f"CONSTRAINT {i+1}: {details['name']}\n")
            f.write(f"Sense: {details['sense']}, RHS: {details['rhs']}\n")
            
            if details['terms']:
                f.write("Terms:\n")
                for term in details['terms']:
                    f.write(f"  {term['coefficient']:+.6f} * {term['variable']}\n")
            
            if details['constant'] != 0:
                f.write(f"Constant: {details['constant']}\n")
            
            f.write("\n" + "-"*30 + "\n\n")
    
    print(f"Constraint details exported to {filename}")
