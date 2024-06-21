import random
import math
from graphs import Grafo as Grafo


def dijkstra(grafo):
    for arista in grafo.aristas:
        arista.distancia = random.randint(0,100)
    GrafoNuevo = Grafo()
    for arista in grafo.aristas:
        distancia =2147483647
        nodito = ""
        for aristita in grafo.aristas:
            if aristita.origen == arista.origen and arista.distancia< distancia:
                distancia = arista.distancia
                nodito = aristita.destino
        GrafoNuevo.agregarArista(arista.origen, nodito)
    
    return GrafoNuevo