import sys
import numpy


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
                self.__rotulos.append(linha[linha.index(" ") + 1:-1])
                temp = linha.split(' "')
                k, v = temp[0], temp[1]
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

    # cria um novo grafo correspondene ao grafo inicial, porém transposto. o retorno é o grafo transposto
    def transposto(self):
        transposto = Grafo.__new__(Grafo)
        transposto.__qtd_vertices = self.__qtd_vertices
        transposto.__qtd_arcos = self.__qtd_arcos
        transposto.__infinito = self.__infinito
        transposto.__rotulos = self.__rotulos.copy()
        transposto.__orientado = self.__orientado

        # matriz de adjacencias transposta
        matriz = [[0] * self.__qtd_vertices for _ in range(self.__qtd_vertices)]
        for v1 in range(self.__qtd_vertices):
            for v2 in range(self.__qtd_vertices):
                matriz[v2][v1] = self.__matriz[v1][v2]
        transposto.__matriz = matriz

        # vetor de adjacencias transposto
        vetor = [[] for _ in range(self.__qtd_vertices)]
        for v1 in range(self.__qtd_vertices):
            for (v2, peso) in self.__vetor[v1]:
                vetor[v2 - 1].append((v1 + 1, peso))
        transposto.__vetor = vetor

        return transposto


# --------------------- Q1 -----------------------------
def visita(grafo, u, C, T, A, F, tempo):
    C[u - 1] = True  # visitado como verdadeiro
    tempo += 1
    T[u - 1] = tempo  # atualiza tempo inicial

    vizinhos = grafo.vizinhos(u)
    for (vizinho, _) in vizinhos:
        if not C[vizinho - 1]:
            A[vizinho - 1] = u
            # visita uma vez para cada vértice
            tempo = visita(grafo, vizinho, C, T, A, F, tempo)

    tempo += 1
    F[u - 1] = tempo  # atualiza tempo final
    return tempo


def busca_profundidade(grafo, ordenado=None):
    qtdVertices = grafo.qtdVertices()
    # inicializacao
    C = [False] * qtdVertices  # visitado = falso
    T = [grafo.infinito] * qtdVertices  # tempo inicial infinito
    F = [grafo.infinito] * qtdVertices  # tempo inicial infinito
    A = [None] * qtdVertices  # ancestrais nulos
    tempo = 0  # configura tempo de inicio

    if ordenado is not None:
        vertices = [(ordenado[i - 1], i) for i in range(1, qtdVertices + 1)]
        vertices.sort(reverse=True)
        vertices = [u for (_, u) in vertices]
    else:
        vertices = range(1, qtdVertices + 1)

    for u in vertices:
        if not C[u - 1]:
            tempo = visita(grafo, u, C, T, A, F, tempo)

    return C, T, A, F


def componentes_fortemente_conexas(grafo):
    # chama DFS para computar os tempos de término para cada vértice
    # C(visitado), T(tempo inicial), A(ancestral), F(tempo final)
    C, T, A, F = busca_profundidade(grafo)
    # cria grafo transposto
    transposto = grafo.transposto()

    # chama DFS para tranposto
    Ct, Tt, At, Ft = busca_profundidade(transposto, ordenado=F)

    # dicionario com chave determinada pelo indice de pesquisa e array de componentes fortemente conexos no valor
    cfc = {}
    for i in range(len(At)):
        if At[i] is None:
            if (i + 1) not in cfc:
                cfc[i + 1] = [i + 1]
        else:
            indice = i
            while At[indice] is not None:
                indice = At[indice] - 1
            if (indice + 1) not in cfc:
                cfc[indice + 1] = [indice + 1]
            cfc[indice + 1].append(i + 1)

    for c in cfc:
        cfc[c].sort()
        print(", ".join(map(str, cfc[c])))
    return At


# --------------------- Q2 -----------------------------
'''
Entrada: grafo dirigido e não ponderado G=(V, A) e ordena linearmente todos os vértices
tal que se existe um arco (u, v) ∈ A então u aparece antes de v na ordenação.
'''


def ordenacao_topologica(grafo):
    qtdVertices = grafo.qtdVertices()
    foiVisitado = [False] * (qtdVertices + 1)
    ordem = []
    # tempo = 0

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
    # tempo = tempp + 1

    for i in grafo.vizinhos(each):
        if (foiVisitado[each] == False):
            dfs_visit_ot(grafo, each, foiVisitado, ordem)

    ordem.append(each)


# --------------------- Q3 -----------------------------

def prepare_sort(arestas, low, high):
    pivot = arestas[high][2]
    item = low - 1

    for i in range(low, high):
        if arestas[i][2] <= pivot:
            item = item + 1
            (arestas[item], arestas[i]) = (arestas[i], arestas[item])

    (arestas[item + 1], arestas[high]) = (arestas[high], arestas[item + 1])
    return item + 1


def quick_sort(arestas, low, high):
    if low < high:
        pivot = prepare_sort(arestas, low, high)
        quick_sort(arestas, low, pivot - 1)
        quick_sort(arestas, pivot + 1, high)


def array_compare_different(a, b):
    counter = 0
    for i in a:
        for j in b:
            if i == j:
                counter += counter
                break;
    if counter < len(a):
        return True
    else:
        return False


def _subarvore(vertice, subarvores):
    for arvore_i in range(len(subarvores)):
        for u in range(len(subarvores[arvore_i])):
            if subarvores[arvore_i][u] == vertice:
                return arvore_i
    return 0


def kruskal(grafo):
    arestas = []  # elemento de 'arestas[]' é uma tupla (u, v, peso)
    matriz = grafo.matriz()  #
    infinite = grafo.infinito  #
    for i in range(grafo.qtdVertices()):  # constrói arranjo de arestas
        for j in range(grafo.qtdVertices()):  #
            if matriz[i][j] < infinite:  #
                arestas.append((i, j, matriz[i][j]))  #
    quick_sort(arestas, 0, len(arestas) - 1)  # ordena as arestas

    arvoreGM = []
    subarvores = [[] for _ in range(grafo.qtdVertices())]  # elemento de 'subarvores' é uma lista de vertices
    x = []

    for i in range(len(subarvores)):
        subarvores[i].append(i)

    somatoria = 0
    for aresta in arestas:
        xtemp = []
        if array_compare_different(subarvores[aresta[0]], subarvores[aresta[1]]):
            arvoreGM.append(aresta)
            somatoria = somatoria + aresta[2]
            xtemp = subarvores[aresta[0]]
            xtemp.extend(subarvores[aresta[1]])
            x = xtemp
            for y in x:
                subarvores[y] = x

    print_kruskal(arvoreGM, somatoria)


def print_kruskal(arvoreGM, somatoria):
    print(somatoria)
    for aresta in arvoreGM:
        print(aresta[0], aresta[1], sep='-', end=", ")
    print("\b\b", end=" ")
    print()


def a2():
    grafo = Grafo("./tests/caminho_minimo/fln_pequena.net")

    print("\n1 - Componentes fortemente conexas")
    componentes_fortemente_conexas(grafo)
    print("\n2 - Ordenação topologica")
    ordenacao_topologica(grafo)
    print("\n3 - Kruskal")
    kruskal(grafo)
