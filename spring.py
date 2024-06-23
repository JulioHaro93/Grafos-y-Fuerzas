import pygame
import math as mad
import random

class Spring:
    def __init__(self):
        self.c1 = 2.0
        self.c2 = 1.0
        self.c3 = 1.0
        self.c4 = 0.01

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

        forces = {nodo: [0, 0] for nodo in grafo.nodos}
        if grafo.nodos:
            x_coords = [nodo.valorEquis for nodo in grafo.nodos]
            y_coords = [nodo.valorYe for nodo in grafo.nodos]
            center_x = sum(x_coords) / len(grafo.nodos)
            center_y = sum(y_coords) / len(grafo.nodos)
        else:
            center_x, center_y = 450, 300

        offset_x = 450 - center_x
        offset_y = 300 - center_y

        for arista in grafo.aristas:
            u, v = arista.origen, arista.destino

            dx = u.valorEquis - v.valorEquis
            dy = u.valorYe - v.valorYe
            distance = mad.sqrt(dx ** 2 + dy ** 2)
            arista.origen.pos = [u.valorEquis, u.valorYe]
            arista.destino.pos = [v.valorEquis, v.valorYe]
            pygame.draw.line(screen, (155, 155, 155), u.pos, v.pos, 2)
            if distance != 0:
                force = spring.c4 * (distance - spring.c2)
                fx = force * dx / distance
                fy = force * dy / distance

                forces[u][0] -= fx
                forces[u][1] -= fy
                forces[v][0] += fx
                forces[v][1] += fy
        for nodo in grafo.nodos:
            nodo.valorEquis += spring.c3 * forces[nodo][0] + offset_x
            nodo.valorYe += spring.c3 * forces[nodo][1] + offset_y

        
        screen.fill((0, 0, 0))

        for arista in grafo.aristas:
            u, v = arista.origen, arista.destino
            pygame.draw.line(screen, (255, 255, 0), (u.valorEquis, u.valorYe), (v.valorEquis, v.valorYe), 2)

        for nodo in grafo.nodos:
            pygame.draw.circle(screen, (255, 0, 0), (int(nodo.valorEquis), int(nodo.valorYe)), 5)

        pygame.display.flip()
        clock.tick(20)

    pygame.quit()
