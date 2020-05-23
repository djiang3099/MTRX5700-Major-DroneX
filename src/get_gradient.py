import math

def get_grad( lineStart, lineEnd ):
    # y = mx + (-m*x_1 + y_1) 
    # 0 = mx - y + (-m*x_1 + y_1) = Ax + By + C
    m = (lineEnd.y - lineStart.y) / (lineEnd.x - lineStart.x) 
            
    return m