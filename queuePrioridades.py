#!/usr/bin/env python
# _*_ coding: utf8 _*_

from queue import PriorityQueue
import heapq 

class Queue:
    tam = None
    tope = None
    datos = None
    
    def __init__(self, tam):
        self.tam = tam
        self.datos = []
    def push(self, dato):
        if(self.llena()):
            return False
        else:
            self.datos.append(dato)
            return True
    def llena(self):
        if len(self.datos) == self.tam:
            return True
        else:
            return False
    def pop(self):
        if self.vacia():
            return True
        else:
            valor = self.datos.pop()
            return valor
    def vacia(self):
        if len(self.datos) == 0:
            return True
        else:
            return False
    def revisar(self,aristas):
        for arista in aristas:
            if arista.destino.visited == False:
                return True
            else:
                return False
    def heapMinimum(grafo):
        pq = PriorityQueue()
        for arista in grafo.aristas:
            pq.put((arista.distancia,arista))
        return pq
    
class priorityMax:
    def __init__(self) -> None:
        self.heap=[]

    def push(self, item, priority):
        heapq.heappush(self.heap, (-priority, item))
    def pop(self):
        _, item = heapq.heappop(self.heap)
        return item
    def empty(self):
        return len(self.heap) == 0