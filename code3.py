import numpy as np

def kalmanFilter(x, P, measurement, R, motion, Q):
    # Predict
    x = np.dot(motion, x)
    P = np.dot(motion, np.dot(P, motion.T)) + Q
    
    # Update
    K = np.dot(P, np.linalg.inv(P + R))
    x = x + np.dot(K, (measurement - x))
    P = P - np.dot(K, np.dot(P, K.T))
    return x, P

# Inicialização
measurements = np.array([[5., 10.], [6., 8.], [7., 6.], [8., 4.], [9., 2.], [10., 0.]])

x0 = np.array([[4.], [12.]]) # Estado inicial
P = np.array([[10000., 0.], [0., 10000.]]) # Matriz de covariância

motion = np.array([[1., 1.], [0., 1.]]) # Matriz de movimento

R = np.array([[0.1, 0.], [0., 0.1]]) # Matriz de incerteza de medição
Q = np.array([[0.1, 0.], [0., 0.1]]) # Matriz de incerteza de movimento

for i in range(len(measurements)):
    measurement = np.array([[measurements[i][0]], [measurements[i][1]]])
    x, P = kalmanFilter(x0, P, measurement, R, motion, Q)

print("Estado final: \n", x)
print("Matriz de incerteza de movimento: \n", Q, '\n')
print("Matriz de incerteza de medição: \n", R, '\n')
print("Matriz de covariância final: \n", P, '\n')