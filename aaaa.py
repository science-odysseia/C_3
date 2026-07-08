import ast

a = "[3, [1, 2, 3, 4, 5]]"
b = ast.literal_eval(a)

print(b)
print(b[0])
print(b[1])
print(b[1][2])