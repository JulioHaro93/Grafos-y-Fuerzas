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

        # Calcular el área del espacio bidimensional
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

        # Calcular el desplazamiento para centrar el grafo en la pantalla
        offset_x = 450 - center_x  # 450 es la mitad del ancho de la pantalla (900 / 2)
        offset_y = 300 - center_y  # 300 es la mitad de la altura de la pantalla (600 / 2)

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

"""
Esta función utiliza la clase Spring para hacer un dibujito de un grafo, calcula las fuerzas,
la posición, 



def springDibujo(grafo):
    pygame.init()

    screen = pygame.display.set_mode((900, 600))

    clock = pygame.time.Clock()
    running = True

    while running:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        spring = Spring()
        #pos = {nodo: (nodo.valorEquis, nodo.valorYe) for nodo in grafo.nodos}
        x =0
        for x in range(len(grafo.nodos)):
            print(grafo.nodos[x].valorEquis)
            #print(grafo.nodos[x].valorYe)
            grafo.nodos[x].pos = (grafo.nodos[x].valorEquis, grafo.nodos[x].valorYe)
            #print(grafo.nodos[x].pos)
            x+=1
        xCoords=[]
        yCoords = []
        for nodo in grafo.nodos:
            xCoords.append(nodo.pos[0])
            yCoords.append(nodo.pos[1])

            xMin, xMax = min(xCoords), max(xCoords)
            yMin, yMax = min(yCoords), max(yCoords)

            width = xMax - xMin
            height = yMax - yMin
            area = 900*600 #width * height

            print(f"Área del espacio bidimensional: {area}")

            k = spring.c4 * mad.sqrt(area / len(grafo.aristas))
            print(k)
            forces = {nodo: (0, 0) for nodo in grafo.nodos}
            dx=0;dy=0
            for arista in grafo.aristas:
                u, v = arista.origen, arista.destino
                print(u.pos)
                dx = u.pos[0] - v.pos[0]
                #print(dx)
                dy = u.pos[1] - v.pos[1]
                #print(dy)
                arista.distancia = mad.sqrt(((dx)**2)+((dy)**2))

                if arista.distancia != 0:
                    force = spring.fa(arista.distancia, k) - spring.fr(arista.distancia, k)
                    fx = force * dx / arista.distancia
                    fy = force * dy / arista.distancia

                    forces[u] = (forces[u][0] + fx, forces[u][1] + fy)
                    forces[v] = (forces[v][0] - fx, forces[v][1] - fy)

                for nodo in grafo.nodos:
                    nodo.valorEquis += spring.c4 * forces[nodo][0]
                    nodo.valorYe += spring.c4 * forces[nodo][1]
                    nodo.pos = (nodo.valorEquis, nodo.valorYe)
       
        screen.fill("purple")

        pygame.display.flip()
        
        clock.tick(60)  # limits FPS to 60

    

    #pygame.quit()

"""
"""

# Ejemplo de uso
grafo = Grafo()

# Crear nodos con posiciones iniciales aleatorias
nodo1 = Nodo(1, (random.uniform(0, 1), random.uniform(0, 1)))
nodo2 = Nodo(2, (random.uniform(0, 1), random.uniform(0, 1)))
nodo3 = Nodo(3, (random.uniform(0, 1), random.uniform(0, 1)))
nodo4 = Nodo(4, (random.uniform(0, 1), random.uniform(0, 1)))
nodo5 = Nodo(5, (random.uniform(0, 1), random.uniform(0, 1)))

# Agregar nodos al grafo
grafo.agregar_nodo(nodo1)
grafo.agregar_nodo(nodo2)
grafo.agregar_nodo(nodo3)
grafo.agregar_nodo(nodo4)
grafo.agregar_nodo(nodo5)

# Crear aristas entre nodos
grafo.agregar_arista(nodo1, nodo2)
grafo.agregar_arista(nodo1, nodo3)
grafo.agregar_arista(nodo2, nodo4)
grafo.agregar_arista(nodo3, nodo5)

spring_layout(grafo)
Explicación
Definición de las Clases:

Nodo tiene atributos para su identificación, posición y aristas salientes.
Arista conecta dos nodos y tiene un grado y fuerza.
Grafo contiene nodos y aristas, y tiene métodos para agregar nodos y aristas.
Algoritmo de Spring Layout:

Se inicializan posiciones aleatorias para los nodos.
Se calculan las fuerzas entre los nodos basándose en las aristas.
Se actualizan las posiciones de los nodos según las fuerzas calculadas.
Visualización:

Se dibuja el grafo antes y después de actualizar las posiciones de los nodos usando networkx y matplotlib.
Este código implementa el algoritmo de disposición por fuerza de resorte para un grafo y visualiza el grafo antes y después de la actualización de las posiciones de los nodos.


"""