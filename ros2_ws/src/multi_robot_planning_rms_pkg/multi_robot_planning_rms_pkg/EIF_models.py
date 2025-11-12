#!/usr/bin/env python
import numpy as np


""" 
Modelado de estado:
g : Función no lineal de transición de estado
h : Función no lineal de medición
Para la linealización de las funciones para su aproximación a forma gaussiana mediante aproximación de Taylor - orden 1:
G : Jacobiano de g 
H : Jacobiano de h

x ~= g(x,u) ~= ∂g/∂x*x_k-1 + ∂g/∂u*u_k-1 = A*x_k-1 + B*u_k-1 == [x_k; y_k; z_k]
z ~= h(x) ~= ∂h/∂x*x_k-1 = C*x_k-1 == [d1; ...; dn]
"""  

def g_function( mu, u = 0):
    # dimensiones: 3 x 1
    # x_k-1 (mu)= posicion (x_1,y_1,z_1) 
    g = np.array( [ mu[0], 
                    mu[1],
                    mu[2]])
    return g

def G_jacobian(mu):
    # dimesiones: 3 x 3
    G = np.eye(3)
    return G

def h_function_n(mu, pos_b):
    # Funcion para 1 solo baliza
    # h total : dimesiones: n x 1
    # h = [d1, d2, d3, ... , dn] = distancia a cada baliza
    h = np.sqrt( (mu[0] - pos_b[0])**2 + (mu[1] - pos_b[1])**2 + (mu[2] - pos_b[2])**2 )
    return np.array([h])

def H_jacobian_n(mu, pos_b):
    # Jacobiano de h de una sola baliza
    # en total dimensiones: 1 x 3
    H = np.zeros((1, 3))
    for j in range(len(mu)):
        H[0][j] = (mu[j][0] - pos_b[j])/np.sqrt((mu[0][0] - pos_b[0])**2 + (mu[1][0] - pos_b[1])**2 + (mu[2][0] - pos_b[2])**2)
    return H
'''
Modelado del ruido de proceso y de medición:
'''

def R_noise_model(vel_xy_max, vel_z_max, dt):
    """
    Modelo de ruido de proceso:
        Tamaño: 3 x 3 
        El modelo consiste en posicion actual predecida coincide con posicion estimada en tiempo anterior
        se calcula la presicion del modelo considerando el error de prdicción mínimo ~0 (quieto) 
        y error máximo velocidad máxima * tiempo entre predicciones.

        Considerando una distribución gaussiana de ruido de proceso
    """
    errors_min = [0,0,0]
    errors_max = np.array([vel_xy_max*dt, vel_xy_max*dt, vel_z_max*dt])

    # Varianzas al cuadrado
    sigmas = [((e_max - e_min)/4)**2 + 5.0 for e_min, e_max in zip(errors_min, errors_max)]

    # Matriz diagonal de covarianza del ruido de proceso
    R = np.diag(sigmas)
    return R 

def Q_noise_model_n(noise_std):
    """
    Modelo de ruido de medición para una baliza:
        Tamaño: n x n 
        Si se considera que el error de medición es un ruido gaussiano con varianza constante,
        que todas las mediciones son independientes entre sí y que todas las balizas 
        tienen la misma varianza del ruido de medición
    """
    #Q = np.diag([sigma**2 for sigma in noise_std])
    Q = np.array([[noise_std**2]])
    return Q