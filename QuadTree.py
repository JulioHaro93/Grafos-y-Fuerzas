import pygame
import numpy as np
from spring import Spring  # Suponiendo que Spring está correctamente implementado en spring.py
import random
import math as mad

class Rectangulito:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def contains(self, point):
        if (point.valorEquis > self.x - self.w and
            point.valorEquis < self.x + self.w and
            point.valorYe > self.x - self.h and
            point.valorYe < self.y + self.h):
            return True
        else:
            return False

class QuadTree:
    def __init__(self, boundary, capacity=4):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.divided = False
        self.noreste = None
        self.noroeste = None
        self.sudeste = None
        self.sudoeste = None
        self.masa = 1

    def subdivide(self):
        x = self.boundary.x
        y = self.boundary.y
        w = self.boundary.w
        h = self.boundary.h

        ne = Rectangulito(x + w/2, y - h/2, w/2, h/2)
        no = Rectangulito(x - w/2, y - h/2, w/2, h/2)
        se = Rectangulito(x + w/2, y + h/2, w/2, h/2)
        so = Rectangulito(x - w/2, y + h/2, w/2, h/2)

        self.noreste = QuadTree(no, self.capacity)
        self.noroeste = QuadTree(ne, self.capacity)
        self.sudoeste = QuadTree(so, self.capacity)
        self.sudeste = QuadTree(se, self.capacity)
        self.divided = True

    def insert(self, node):
        if not self.boundary.contains(node):
            return False

        if len(self.points) < self.capacity:
            self.points.append(node)
            return True
        else:
            if not self.divided:
                self.subdivide()

            if self.noreste.insert(node):
                return True
            elif self.noroeste.insert(node):
                return True
            elif self.sudeste.insert(node):
                return True
            elif self.sudoeste.insert(node):
                return True

    def forceBarnesHut(self, node, spring):
        G = 1.0
        theta = 0.8

        def calculateForce(node1, node2):
            force =0
            if  hasattr(node2, 'valorEquis') == True:
                distance_vector = np.array([node1.valorEquis - node2.valorEquis, node1.valorYe - node2.valorYe])
                distance = np.linalg.norm(distance_vector)
                magnitude = G * node1.masa * node2.masa / (distance ** 2 + 0.1)
                force = magnitude * distance_vector / distance
                return force
            else:
                return force

        def calculateForceRecursive(quad, node):
            if not quad:
                return np.array([0.0, 0.0])

            if not quad.divided:
                force = np.array([0.0, 0.0])
                for p in quad.points:
                    if p != node:
                        #print(p)
                        force += calculateForce(node,p)
                return force
            else:
                dist = np.linalg.norm(np.array([node.valorEquis, node.valorYe]) - np.array([quad.boundary.x, quad.boundary.y]))
                if quad.boundary.w / dist < theta:
                    force = calculateForce(node, quad)
                    return force
                else:
                    force = np.array([0.0, 0.0])
                    force += calculateForceRecursive(quad.noreste, node)
                    force += calculateForceRecursive(quad.noroeste, node)
                    force += calculateForceRecursive(quad.sudeste, node)
                    force += calculateForceRecursive(quad.sudoeste, node)
                    return force

        total_force = calculateForceRecursive(self, node)
        return abs(total_force)

    def show(self, screen):
        pygame.draw.rect(screen, (255, 255, 255),
                         pygame.Rect(self.boundary.x - self.boundary.w,
                                     self.boundary.y - self.boundary.h,
                                     self.boundary.w * 2,
                                     self.boundary.h * 2), 1)

        if self.divided:
            self.noroeste.show(screen)
            self.noreste.show(screen)
            self.sudoeste.show(screen)
            self.sudeste.show(screen)

        for node in self.points:
            pygame.draw.circle(screen, (255, 0, 0), (int(node.valorEquis), int(node.valorYe)), 2)
def fuerzaBruta(grafo):
    sumaDeFuerzas = 0
    i = 0
    for arista in grafo.aristas:
        d = mad.sqrt(abs((arista.origen.valorEquis**2) - (arista.destino.valorYe**2)))
        #print(d)
        # le puse 2 porque considero la masa de 1 jajaja porque phisics, una unidad galáxica
        if d != 0:
            f = 2/d
            grafo.aristas[i].fuerza = f
            i+=1
            #print(arista.fuerza)
            sumaDeFuerzas += arista.fuerza
        else:
            continue
    return sumaDeFuerzas

def main(grafo):
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    clock = pygame.time.Clock()
    running = True
    fuente = pygame.font.Font(None, 20)
    message = str("Suma de fuerzas O(n²): " + str(fuerzaBruta(grafo)))
    fuercilla = fuerzaBruta(grafo)
    message2 = str("Suma de Fuerzas O(nlog(n))")
    mensaje = fuente.render(message, 1, (255, 255, 255))
    spring = Spring()
    boundary = Rectangulito(400, 300, 400, 300)
    quadtree = QuadTree(boundary)
    forceBH =0
    
    for nodo in grafo.nodos:
        quadtree.insert(nodo)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((0, 0, 0))

        
        for nodo in grafo.nodos:
            forces = quadtree.forceBarnesHut(nodo, spring)
            #print(forces)
            forceBH += forces
            if forceBH[0] > fuercilla:
                break
        message2 = str("Suma de Fuerzas O(nlog(n))"+ str(forceBH))
        mensaje2 = fuente.render(message2, 1, (255, 255, 255))
        for arista in grafo.aristas:
            u, v = arista.origen, arista.destino
            pygame.draw.line(screen, (155, 155, 155), u.pos, v.pos, 2)

        
        for nodo in grafo.nodos:
            if not np.isnan(nodo.pos[0]) and not np.isnan(nodo.pos[1]):
                pygame.draw.circle(screen, (255, 255, 0), (int(nodo.valorEquis), int(nodo.valorYe)), 3)

        quadtree.show(screen)
        warning1 = "La suma que calcula O(n²)" 
        warning2= "se realiza antes de pygame"
        warning3 = "por esa razón aparece estática"
        warningWarning ="Warning:"
        mensaje3 = fuente.render(warning1, 1, (255, 255, 255))
        mensaje4 = fuente.render(warning2,1,(255,255,255))
        mensaje5 = fuente.render(warning3,1,(255,255,255))
        mensaje6= fuente.render(warningWarning,1,(255,255,0))
        screen.blit(mensaje4,(600,420))
        screen.blit(mensaje5,(600,440))
        screen.blit(mensaje6, (600, 380))
        screen.blit(mensaje3, (600, 400))
        screen.blit(mensaje, (600, 500))
        screen.blit(mensaje2, (600, 520))
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

