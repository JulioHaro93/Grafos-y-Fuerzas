import graphs as grafo
from queue import Queue as que
from graphs import Grafo as Grafo
from dijkstra import dijkstra as dk
from kruskalPrim import KruskalPrim as pk
import random
import spring as sprincito
from QuadTree import main
import math as mad

grafo = Grafo()
        
grafoMalla30 = Grafo().grafoMalla(100,1,False)

x=0
for x in range(len(grafoMalla30.nodos)):
    grafoMalla30.nodos[x].valorEquis = random.randint(0,500)
    #print(grafoMalla30.nodos[x].valorEquis)
    grafoMalla30.nodos[x].valorYe = random.randint(0,500)
    #print(grafoMalla30.nodos[x].valorYe)
    grafoMalla30.nodos[x].pos = (grafoMalla30.nodos[x].valorEquis,grafoMalla30.nodos[x].valorYe)
    x+=1

i=0
for arista in grafo.aristas:
    d = mad.sqrt(abs((arista[0].valorEquis**2) - (arista[1].valorYe**2)))
    #print(d)
    # le puse 2 porque considero la masa de 1 jajaja porque phisics, una unidad galáxica
    f = 2/d
    grafo.aristas[i].fuerza = f
    i+=1

spring = sprincito
spring.springDibujo(grafoMalla30)

#main(grafoMalla30)
"""

Testings de otras cosillas.

krusk = pk.kruskal(grafoMalla30)

grafo.generaGephi(krusk[0], "K-Chico")
print("________________________________________")
kruski = pk.kruskalInv(grafoMalla30)
grafo.generaGephi(kruski[0], "KI-Chico")
print("________________________________________")
#prim = pk.prim(grafoMalla30)
#print(prim)
#grafo.generaGephi(prim[0],"prim-Chico")

grafoMalla100 = Grafo().grafoBarabasiAlbert(100, 5, False)
for arista in grafoMalla100.aristas:
    arista.distancia = random.randint(0,10000)
krusk = pk.kruskal(grafoMalla100)

grafo.generaGephi(krusk[0], "K-Mediano")
print("________________________________________")
kruski = pk.kruskalInv(grafoMalla100)
grafo.generaGephi(kruski[0], "KI-Mediano")
print("________________________________________")
#prim = pk.prim(grafoMalla100)
#print(prim)
#grafo.generaGephi(prim[0],"prim-Mediano")


grafoMalla500 = Grafo().grafoBarabasiAlbert(500, 5, False)
for arista in grafoMalla500.aristas:
    arista.distancia = random.randint(0,10000)
krusk = pk.kruskal(grafoMalla500)

grafo.generaGephi(krusk[0], "K-Grande")
print("________________________________________")
kruski = pk.kruskalInv(grafoMalla500)
grafo.generaGephi(kruski[0], "KI-Grande")
print("________________________________________")
#prim = pk.prim(grafoMalla500)
#print(prim)
#grafo.generaGephi(prim[0],"prim-Grande")

"""
