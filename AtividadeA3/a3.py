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
    caminho_arquivo = input("Entre com caminho do arquivo de teste: ")
    grafo = Grafo(caminho_arquivo, sys.maxsize)


# --------------------- Q2 -----------------------------
def hopcroft_karp():
    print("Hopcroft Karp")
    caminho_arquivo = input("Entre com caminho do arquivo de teste: ")
    grafo = Grafo(caminho_arquivo, sys.maxsize)


# --------------------- Q3 -----------------------------
# conjunto enumerado de potencia
def potencia(elementos):
    # https://stackoverflow.com/questions/1482308/how-to-get-all-subsets-of-a-set-powerset/1482320#1482320
    qtdElementos = len(elementos)
    potencias = {}

    for i in range(1 << qtdElementos):
        potencias[i] = set(
            [elementos[j] for j in range(qtdElementos) if (i & (1 << j))])

    return potencias


# encontra numero do elemento atribuido nas potencias
def f(subconjunto, potencias):
    for i in potencias:
        if potencias[i] == subconjunto:
            return i

    return 0


def coloracao_vertices():
    print("Coloração de Vértices")
    # arquivos que foram utilizados para teste: ./tests/coloracao/ pentagrama, pentagono, cor3
    caminho_arquivo = input("Entre com caminho do arquivo de teste: ")
    grafo = Grafo(caminho_arquivo, sys.maxsize)

    # lawler
    X = [grafo.pesoMinMax] * (1 << grafo.qtdVertices())
    X[0] = 0

    S_ = potencia(list(range(1, grafo.qtdVertices()+1)))

    for s in S_:
        S = S_[s]
        S_potencia = potencia(list(S))
        independentes = []

        # encontra conjuntos independentes
        for sp in S_potencia:
            Ssp = S_potencia[sp]
            try:
                for u in Ssp:
                    for v in Ssp:
                        if (grafo.haAresta(u, v) or grafo.haAresta(v, u)) and u != v:
                            raise Exception()
            except:
                continue
            independentes.append(Ssp)

        for I in independentes:
            i = f(S.difference(I), S_)
            if X[i] + 1 < X[s]:
                X[s] = X[i] + 1

    grupo_cores = [[] for _ in range(X[-1])]
    for u in range(1, grafo.qtdVertices()+1):
        for grupo in grupo_cores:
            try:
                for v in grupo:
                    if grafo.haAresta(u, v) or grafo.haAresta(v, u):
                        raise Exception()
            except:
                continue

            grupo.append(u)
            break

    print_coloracao(X, grupo_cores)


def print_coloracao(X, grupo_cores):
    print(f"Coloração mínima: {X[-1]}")
    for i in range(len(grupo_cores)):
        string = [str(e) for e in grupo_cores[i]]
        print(f"Cromático {i + 1}: {', '.join(string)}")


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
