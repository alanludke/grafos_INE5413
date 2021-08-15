import sys


# grafo não-dirigido e ponderado G(V, E, w)
# V é o conjunto de vértices
# E é o conjunto de arestas
# w : E → R é a função que mapeia o peso de cada aresta {u, v} ∈ E
class Grafo:
    def __init__(self, nomeArquivo):
        self.__infinito = sys.maxsize
        self.__vertices = {}
        self.__rotulos = []
        with open(nomeArquivo) as arquivo:
            qtdVertices = int(arquivo.readline().split()[-1])
            self.__qtd_vertices = qtdVertices
            for _ in range(qtdVertices):
                linha = arquivo.readline()
                self.__rotulos.append(linha[linha.index(" ")+1:-1])
                temp = linha.split(' "')
                k,v = temp[0],temp[1]
                self.__vertices[int(k)] = v.rstrip("\n").strip('\"')

            self.__vetor = [[] for _ in range(qtdVertices)]
            self.__matriz = [[self.__infinito] *
                             qtdVertices for _ in range(qtdVertices)]
            self.__orientado = arquivo.readline() == "*arcs\n"  # pula linha que contem *edges/arcs

            arcos = arquivo.readlines()
            self.__qtd_arcos = len(arcos)
            for arco in arcos:
                v1, v2, peso = arco.split()
                v1 = int(v1) - 1
                v2 = int(v2) - 1
                peso = float(peso)

                self.__matriz[v1][v2] = peso
                self.__vetor[v1].append((v2 + 1, peso))
                if not self.__orientado:
                    self.__vetor[v2].append((v1 + 1, peso))
                    self.__matriz[v2][v1] = peso

    @property
    def infinito(self):
        return self.__infinito

    # retorna a quantidade de vértices;
    def qtdVertices(self):
        return self.__qtd_vertices

    # retorna o vetor de vértices
    def getVertices(self):
        return self.__vertices

    # retorna a matriz
    def matriz(self):
        return self.__matriz

    # retorna a quantidade de arestas
    def qtdArcos(self):
        return self.__qtd_arcos

    # retorna vertices que ligam na aresta
    def vertice(self, arco):
        return self.__arcos[arco][0]

    def arcos(self):
        return self.__arcos

    # retorna o grau do vértice v
    def grau(self, v):
        return len(self.__vetor[v - 1])

    # retorna o rótulo do vértice v
    def rotulos(self):
        return self.__rotulos

    # retorna o rótulo do vértice v
    def rotulo(self, v):
        return self.__rotulos[v - 1]

    # retorna lista de tuplas com os vizinhos do vértice v
    def vizinhos(self, v):
        return self.__vetor[v - 1]

    # se {u, v} ∈ E, retorna verdadeiro; se não existir, retorna falso
    def haAresta(self, u, v):
        return self.__matriz[u - 1][v - 1] != self.__infinito

    # se {u, v} ∈ E, retorna o peso da aresta {u, v}; se não existir, retorna um valor infinito positivo 1
    def peso(self, u, v):
        return self.__matriz[u - 1][v - 1]

    def transposto(self):
        transposto = Grafo.__new__(Grafo)
        transposto.__qtd_vertices = self.__qtd_vertices
        transposto.__qtd_arcos = self.__qtd_arcos
        transposto.__infinito = self.__infinito
        transposto.__rotulos = self.__rotulos.copy()
        transposto.__orientado = self.__orientado

        matriz = [[0] * self.__qtd_vertices for _ in range(self.__qtd_vertices)]
        for v1 in range(self.__qtd_vertices):
            for v2 in range(self.__qtd_vertices):
                matriz[v2][v1] = self.__matriz[v1][v2]
        transposto.__matriz = matriz

        vetor = [[] for _ in range(self.__qtd_vertices)]
        for v1 in range(self.__qtd_vertices):
            for (v2, peso) in self.__vetor[v1]:
                vetor[v2 - 1].append((v1 + 1, peso))
        transposto.__vetor = vetor

        return transposto


def visita(grafo, u, C, T, A, F, tempo):
    C[u - 1] = True
    tempo += 1
    T[u - 1] = tempo

    vizinhos = grafo.vizinhos(u)
    for (vizinho, _) in vizinhos:
        if not C[vizinho - 1]:
            A[vizinho - 1] = u
            tempo = visita(grafo, vizinho, C, T, A, F, tempo)

    tempo += 1
    F[u - 1] = tempo
    return tempo


def busca_profundidade(grafo, ordenado=None):
    qtdVertices = grafo.qtdVertices()
    C = [False] * qtdVertices
    T = [grafo.infinito] * qtdVertices
    F = [grafo.infinito] * qtdVertices
    A = [None] * qtdVertices
    tempo = 0

    if ordenado is not None:
        vertices = [(ordenado[i - 1], i) for i in range(1, qtdVertices + 1)]
        vertices.sort(reverse=True)
        vertices = [u for (_, u) in vertices]
    else:
        vertices = range(1, qtdVertices + 1)

    for u in vertices:
        if not C[u - 1]:
            tempo = visita(grafo, u, C, T, A, F, tempo)

    return (C, T, A, F)


def componentes_fortemente_conexas(grafo):
    C, T, A, F = busca_profundidade(grafo)
    transposto = grafo.transposto()
    print(grafo.matriz)
    print(transposto.matriz)

    Ct, Tt, At, Ft = busca_profundidade(transposto, ordenado=F)

    cfc = {}
    for i in range(len(At)):
        if At[i] is None:
            if(i+1) not in cfc:
                cfc[i+1] = [i+1]
        else:
            indice = i
            while At[indice] is not None:
                indice = At[indice] - 1
            if (indice+1) not in cfc:
                cfc[indice + 1] = [indice + 1]
            cfc[indice + 1].append(i+1)

    for c in cfc:
        cfc[c].sort()
        print(", ".join(map(str, cfc[c])))
    return At

'''
Entrada: grafo dirigido e não ponderado G=(V, A) e ordena linearmente todos os vértices
tal que se existe um arco (u, v) ∈ A então u aparece antes de v na ordenação.
'''
def ordenacao_topologica(grafo):
    qtdVertices = grafo.qtdVertices()
    foiVisitado = [False] * (qtdVertices+1)
    ordem = []
    #tempo = 0

    for each in grafo.getVertices():
        if (foiVisitado[each] == False):
            dfs_visit_ot(grafo, each, foiVisitado, ordem)

    for i, each in enumerate(ordem):
        print(grafo.rotulo(each), end='')
        if i < len(ordem) - 1:
            print(' > ', end='')
    print()

def dfs_visit_ot(grafo, each, foiVisitado, ordem):
    foiVisitado[each] = True
    #tempo = tempp + 1

    for i in grafo.vizinhos(each):
         if (foiVisitado[each] == False):
             dfs_visit_ot(grafo, each, foiVisitado, ordem)

    ordem.append(each)

def a2():
    # exe1
    # grafo = Grafo("./tests/mesa_dfs.net")
    # print("Componentes fortemente conexas")
    # componentes_fortemente_conexas(grafo)

    #exe2
    grafo = Grafo("./tests/dirigidos/manha.net")
    ordenacao_topologica(grafo)
