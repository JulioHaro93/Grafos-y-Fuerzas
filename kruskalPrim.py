from queuePrioridades import Queue as priority
from queuePrioridades import priorityMax as max
import random
from graphs import Grafo as Grafo
from queue import PriorityQueue
from dfs import dfs_r as dfs


class KruskalPrim:
    def __init__(self) -> None:
        pass
    def kruskal(grafo):
        grafoNuevo = priority.heapMinimum(grafo)
        grafito = Grafo()
        MST =0
        while not grafoNuevo.empty():
            nueva = grafoNuevo.get()
            for arista in grafo.aristas:
                if arista.origen == nueva[1].origen:
                    MST += nueva[0]
                    grafito.agregarArista(arista.origen, nueva[1].destino)
        return grafito, MST
    
    def kruskalInv(grafo):
        MST=0
        grafito = Grafo()
        maxim = max()
        for arista in grafo.aristas:
            maxim.push(arista, arista.distancia)
        
        while not maxim.empty():
            nueva = maxim.pop()
            #print(nueva.distancia)
            for arista in grafo.aristas:
                if arista.origen == nueva.origen:
                    #print(nueva)
                    MST += arista.distancia
                    grafito.agregarArista(arista.origen, nueva.destino)
        result = [x for x in grafo.aristas if x not in grafito.aristas]
        for arista in result:
            pass
        return grafito, MST
    
    from queue import PriorityQueue

    def prim(grafo):
        MST = 0
        q = PriorityQueue()
        grafillo = Grafo()
        nodoInicial= grafo.obtenerNodoArbitrario(len(grafo.aristas)-1)        
        for arista in grafo.aristas:
            q.put(arista)
                
        while not q.empty():
            key = []
            salientes = []
            arista = q.get()
            arista.origen.visited = True
            index = grafo.aristas.index(arista)
            x=0
            for x in range(len(grafo.aristas)):
                if arista.origen == grafo.aristas[x].origen:
                    key.append(grafo.aristas[x].distancia)
                    salientes.append((grafo.aristas[x], grafo.aristas[x].distancia))
            if key:
                MST += min(key)
            if salientes:
                grafillo.aristas.append(min(salientes, key=lambda x: x[1])[0])
            key.clear()
            salientes.clear()
            
            x+=1
        return grafillo, MST