import math 

# 
x2 = -5.146970789999955
y2 = -10.652939760000038
# 
x1 = -548.95570215
y1 = -213.83008141
# 
x=-1204.72283593
dx= x - x1
total_y = y1 - y2
total_x = x1 - x2
dy = (total_y/total_x)*dx
y = y1 + dy
print(y)
# new length 
dx_new = abs(x-x2)
dy_new = abs(y-y2)
new_length = math.sqrt(dx_new**2 + dy_new**2)
print(new_length)

# --------------------------------

# x = 815.43725427
# y = 316.29216000

# dx = 820.16009019
# dy = 319.29216000

# print('New x is: ' + str(x-dx))
# print('New y is: ' + str(y-dy))