import matplotlib.pyplot as plt
import random
import networkx as nx
import math as mad
import pygame
import numpy as np

class Spring:
    def __init__(self):
        self.c1=2.0
        self.c2=1.0
        self.c3=1.0
        self.c4=0.0001
    def fa(self,d, k):
        return pow(d,2)/k
    def fr(self,d,k):
        return -pow(k,2)/d

class Spring:
    def __init__(self):
        self.c4 = 0.1

    def fa(self, d, k):
        return pow(d, 2) / k

    def fr(self, d, k):
        return -pow(k, 2) / d

def springDibujo(grafo):
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    clock = pygame.time.Clock()
    running = True

    spring = Spring()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        forces = {nodo: (0, 0) for nodo in grafo.nodos}
        
        xCoords = [nodo.valorEquis for nodo in grafo.nodos]
        yCoords = [nodo.valorYe for nodo in grafo.nodos]
        xMin, xMax = min(xCoords), max(xCoords)
        yMin, yMax = min(yCoords), max(yCoords)
        width = xMax - xMin
        height = yMax - yMin
        area = width * height

        k = spring.c4 * mad.sqrt(area / len(grafo.aristas))

        # Calcular el centro del grafo
        center_x = (xMin + xMax) / 2
        center_y = (yMin + yMax) / 2

        offset_x = 450 - center_x 
        offset_y = 300 - center_y  

        for nodo in grafo.nodos:
            nodo.valorEquis += offset_x
            nodo.valorYe += offset_y
            nodo.pos = (nodo.valorEquis, nodo.valorYe)

        for arista in grafo.aristas:
            u, v = arista.origen, arista.destino

            dx = u.valorEquis - v.valorEquis
            dy = u.valorYe - v.valorYe
            arista.distancia = mad.sqrt(dx**2 + dy**2)

            if arista.distancia != 0:
                force = spring.fa(arista.distancia, k) - spring.fr(arista.distancia, k)
                fx = force * dx / arista.distancia
                fy = force * dy / arista.distancia

                forces[u] = (forces[u][0] + fx, forces[u][1] + fy)
                forces[v] = (forces[v][0] - fx, forces[v][1] - fy)

        for nodo in grafo.nodos:
            nodo.valorEquis += spring.c4 * nodo.valorEquis/forces[nodo][0]
            nodo.valorYe += spring.c4 * nodo.valorYe/forces[nodo][1]
            nodo.pos = (nodo.valorEquis, nodo.valorYe)

        screen.fill('blueviolet')

        for arista in grafo.aristas:
            u, v = arista.origen, arista.destino
            pygame.draw.line(screen, (255, 255, 255), u.pos, v.pos, 2)

        for nodo in grafo.nodos:
            if not np.isnan(nodo.pos[0]) and not np.isnan(nodo.pos[1]):
                enterito = int(nodo.pos[0])
                enterote = int(nodo.pos[1])
                pygame.draw.circle(screen, (255, 0, 0), (enterito, enterote), 1)

        pygame.display.flip()
        clock.tick(200)

