from graphs import Grafo as Grafo
from graphs import Nodo as Nodo

def dfs_r(self, grafo, u, v, grafito):
        aristas = grafo.aristas
        if(v < len(aristas)):
            aristas[u].origen.visited = True
            
            for arista in aristas:
                if arista.origen == aristas[u].origen and not arista.destino.visited:
                    grafito.agregarArista(arista.origen, arista.destino)
                    u = v
                    self.dfs_r(grafo, u, v+1, grafito)
        return grafito
    
def dfs_i_malla(self, grafo, inicio):
    stack = [grafo.aristas[inicio]]
    grafillo = Grafo()
    stack = [grafo.aristas[inicio]]
    grafillo = Grafo()
    nodito = Nodo(0,0)
    grafo.aristas[inicio].origen.visited = True
    try:
        while stack:
            nodo_actual = stack.pop()
            if(nodo_actual is not None and nodo_actual != grafo.nodos[len(grafo.nodos)-1][0]):
                nodo_actual= nodo_actual
            for arista in grafo.aristas:
                nodo_actual.toString(nodo_actual)
                if not arista.destino.visited:
                    arista.destino.visited = True
                    stack.append(arista.destino)
                    grafillo.agregarArista(nodo_actual.origen, arista.destino)

        for nodo in grafo.nodos:
            nodo.visited = False
    except:
        print("Se ha leído todo el grafo")
    return grafillo

def dfs_i(self, grafo, inicio):
    
    grafillo = Grafo()
    nodito = Nodo(0,0)
    grafo.aristas[inicio].origen.visited = True
    nodo = grafo.aristas[inicio].origen
    stack = [nodo]
    while stack:
        nodo_actual = stack.pop()
        if(nodo_actual.visited == True):
            for arista in grafo.aristas:
                nodo_actual.toString(nodo_actual)
                if not arista.destino.visited:
                    arista.destino.visited = True
                    stack.append(arista.destino)
                    grafillo.agregarArista(nodo_actual, arista.destino)
        print("intentito")
    return grafillo
