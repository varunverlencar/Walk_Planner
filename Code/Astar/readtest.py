with open('footstep_f.txt') as f:
    path = []
    for line in f:
        line = line.split() # to deal with blank 
        if line:            # lines (ie skip them)
            line = [float(i) for i in line]
            path.append(line)
print path
# Result in the form of [[0.0, 0.0, 0.0, 0.0], [3.5, 4.5, 0.0, 0.0], [4.0, 5.5, 0.0, 2.5], [2.5, 2.5, 0.0, 0.0]]

