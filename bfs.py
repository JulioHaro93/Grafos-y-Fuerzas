import graphs as grafos
Grafo = grafos.Grafo()
Nodo = grafos.Nodo()
import queue_1 as que
Queue = que.Queue()
Arista = grafos.Arista()

def bfs(self, grafo, inicio):
        grafitoBFS = Grafo()
        for arista in grafo.aristas:
            arista.destino.visited = False; arista.origen.visited = False
        q = Queue(len(grafo.aristas))
        nodito = grafo.aristas[inicio].origen
        nodo = Nodo(0,0)
        nodito.visited = True
        q.push(nodito)
        while len(q.datos) != 0:
            nodoActual = q.pop()
            destino = nodoActual
            lista = grafo.aristas
            i =0
            if(i < q.tam):
                for arista in grafo.aristas:
                    if(arista.destino.visited == False):
                        u = arista.destino
                        u.visited = True
                        q.push(u)
                        arista = Arista(arista.origen, arista.destino)
                        grafitoBFS.agregarArista(arista.origen, arista.destino)
            else:
                grafo.aristas[i].destino.visited = True
                print("Se ha completado el BFS")
        return grafitoBFS

