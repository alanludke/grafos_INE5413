import sys


class Grafo:
    def __init__(self, nomeArquivo, pesoMinMax):
        self.__peso_min_max = pesoMinMax
        self.__rotulos = []
        with open(nomeArquivo) as arquivo:
            qtdVertices = int(arquivo.readline().split()[-1])
            self.__qtd_vertices = qtdVertices
            for _ in range(qtdVertices):
                linha = arquivo.readline()
                self.__rotulos.append(linha[linha.index(" ") + 1:-1])

            self.__vetor = [[] for _ in range(qtdVertices)]
            self.__matriz = [[self.__peso_min_max] *
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
    def pesoMinMax(self):
        return self.__peso_min_max

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
        return self.__matriz[u - 1][v - 1] != self.__peso_min_max

    # se {u, v} ∈ E, retorna o peso da aresta {u, v}; se não existir, retorna um valor infinito positivo 1
    def peso(self, u, v):
        return self.__matriz[u - 1][v - 1]

    # cria um novo grafo correspondene ao grafo inicial, porém transposto. o retorno é o grafo transposto
    def transposto(self):
        transposto = Grafo.__new__(Grafo)
        transposto.__qtd_vertices = self.__qtd_vertices
        transposto.__qtd_arcos = self.__qtd_arcos
        transposto.__peso_min_max = self.__peso_min_max
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
def edmonds_karp():
    print("Edmonds Karp")


# --------------------- Q2 -----------------------------
def hopcroft_karp():
    print("Hopcroft Karp")


# --------------------- Q3 -----------------------------
def coloracao_vertices():
    print("Coloração de Vértices")


def a3():
    print("1 - Edmonds Karp\n2 - Hopcroft Karp\n3 - Coloração de Vértices")
    opcao = int(input("--> "))

    if opcao == 1:
        edmonds_karp()
    elif opcao == 2:
        hopcroft_karp()
    elif opcao == 3:
        coloracao_vertices()
    else:
        print("Opção inválida")
        a3()
