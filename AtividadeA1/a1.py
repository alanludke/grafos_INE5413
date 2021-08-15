import sys
import heapq


# grafo não-dirigido e ponderado G(V, E, w)
# V é o conjunto de vértices
# E é o conjunto de arestas
# w : E → R é a função que mapeia o peso de cada aresta {u, v} ∈ E
class Grafo:
    def __init__(self, nomeArquivo):
        self.__infinito = sys.maxsize
        self.__rotulos = []
        self.__qtd_vertices = 0
        self.__qtd_arestas = 0
        self.__vetor = []
        self.__matriz = [[]]
        self.lerArquivo(nomeArquivo)

    @property
    def infinito(self):
        return self.__infinito

    # retorna a quantidade de vértices;
    def qtdVertices(self):
        return self.__qtd_vertices

    # retorna o a matriz
    def matriz(self):
        return self.__matriz

    # retorna a quantidade de arestas
    def qtdArestas(self):
        return self.__qtd_arestas

    # retorna vertices que ligam na aresta
    def vertice(self, aresta):
        return self.__arestas[aresta][0]

    def arestas(self):
        return self.__arestas

    # retorna o grau do vértice v
    def grau(self, v):
        return len(self.__vetor[v - 1])

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

    '''
    deve carregar um grafo a partir de um arquivo no formato abaixo. (./tests/test0.net pode ser utilizado para primeiro teste)
    *vertices n
    1 rotulo_de_1
    2 rotulo_de_2
    ...
    n label_de_n
    *edges
    a b valor_do_peso
    a c valor_do_peso
    ...
    '''

    def lerArquivo(self, nomeArquivo):
        with open(nomeArquivo) as arquivo:
            qtdVertices = int(arquivo.readline().split()[-1])
            self.__qtd_vertices = qtdVertices
            for _ in range(qtdVertices):
                self.__rotulos.append(arquivo.readline().split()[1])

            self.__vetor = [[] for _ in range(qtdVertices)]
            self.__matriz = [[self.__infinito] *
                             qtdVertices for _ in range(qtdVertices)]
            arquivo.readline()  # pula linha que contem *edges

            arestas = arquivo.readlines()
            self.__qtd_arestas = len(arestas)
            self.__arestas = []
            for aresta in arestas:
                v1, v2, peso = aresta.split()
                self.__arestas.append([int(v1), int(v2)])
                v1 = int(v1) - 1
                v2 = int(v2) - 1
                peso = float(peso)

                self.__matriz[v1][v2] = self.__matriz[v2][v1] = peso
                self.__vetor[v1].append((v2 + 1, peso))
                self.__vetor[v2].append((v1 + 1, peso))


def representacao(grafo):
    print(f"Quantidade de vertices: {grafo.qtdVertices()}")
    print(f"Quantidade de arestas: {grafo.qtdArestas()}")

    grau = int(input("Informe vértice para capturar grau: "))
    print(f"Grau: {grafo.grau(grau)}")

    rotulo = int(input("Informe vértice para capturar rótulo: "))
    print(f"Rotulo: {grafo.rotulo(rotulo)}")

    vizinhos = int(input("Informe vértice para capturar vizinhos: "))
    print(f"Vizinhos: {grafo.vizinhos(vizinhos)}")

    v1, v2 = map(int, input(
        "Informe vértices para verificar existencia de aresta e peso: ").split())
    print(f"Há aresta? {grafo.haAresta(v1, v2)}")
    print(f"Peso: {grafo.peso(v1, v2)}")


def buscas(grafo, indiceVertice):
    # inicializacao
    # array de vertices conhecidos como falso
    visitados = [False] * (grafo.qtdVertices() + 1)
    # distancia infinita para todos
    distanciaInicial = [grafo.infinito] * (grafo.qtdVertices() + 1)
    ancestral = [None] * (grafo.qtdVertices() + 1)

    visitados[indiceVertice] = True  # vertice de onde partiu a busca
    distanciaInicial[indiceVertice] = 0  # distancia do vertice inicial
    fila = [indiceVertice]  # cria fila e adicionado vertice inicial
    listaNiveis = {0: [indiceVertice]}  # cria dicionario dos niveis

    while fila:  # enquanto lista nao estiver fazia, faca:
        verticeAtual = fila.pop(0)  # desenfileira 1 posicao
        vizinhos = [vizinho[0] for vizinho in
                    grafo.vizinhos(verticeAtual)]  # captura vertices vizinhos do indice atual, sem peso
        for vizinho in vizinhos:  # percorre vizinhos do vertice
            if not visitados[vizinho]:  # se o vizinho ainda nao foi visitado:
                fila.append(vizinho)  # adiciona vertice vizinho na fila
                visitados[vizinho] = True  # vertice do vizinho foi conhecido
                distanciaInicial[vizinho] = distanciaInicial[verticeAtual] + 1
                ancestral[vizinho] = verticeAtual

                # cria/atualiza dicionario de vizinhos do nivel
                vizinhosNivel = listaNiveis.get(distanciaInicial[vizinho], [])
                vizinhosNivel.append(vizinho)
                listaNiveis.update({distanciaInicial[vizinho]: vizinhosNivel})

    # percorre dicionario imprimindo o nivel e a listagem de vertices encontrados
    print("\nBusca em largura:")
    for chave, valor in listaNiveis.items():
        print(str(chave) + ":", str(valor)[1:-1])

    return distanciaInicial, ancestral


def removeDuplicatas(lista):
    seen = set()
    seen_add = seen.add
    return [x for x in lista if not (x in seen or seen_add(x))]


def dijkstra(grafo, indiceVertice):
    # inicializacao
    # distancia infinita para todos
    distanciaInicial = [grafo.infinito] * (grafo.qtdVertices() + 1)
    distanciaInicial[indiceVertice - 1] = 0  # distancia do vertice inicial
    # todos ancestrais sao nulos
    ancestral = [None] * (grafo.qtdVertices() + 1)
    # ancestral do vértice inicial é ele mesmo
    ancestral[indiceVertice - 1] = indiceVertice

    DISTANCIA, INDICE, VALIDO = 0, 1, 2
    # definir heap min, "senhas" minimas exceto o vertice inicial --- O(n)
    heap = [[0, indiceVertice, True]] + [[distanciaInicial[i], i, True]
                                         for i in range(len(distanciaInicial)) if i != (indiceVertice - 1)]

    while heap:  # enquanto heap não estiver vazia
        # atender a menor "senha", extractmin --- O(log n)
        verticeAtual = heapq.heappop(heap)

        for vizinho in grafo.vizinhos(verticeAtual[INDICE]):
            tentativaNovaDistancia = distanciaInicial[verticeAtual[INDICE] - 1] \
                                     + grafo.peso(verticeAtual[INDICE], vizinho[0])

            if distanciaInicial[vizinho[0] - 1] > tentativaNovaDistancia:
                # decrementa chave --- O(log n)
                distanciaInicial[vizinho[0] - 1] = tentativaNovaDistancia
                ancestral[vizinho[0] - 1] = verticeAtual[INDICE]

                for i in range(len(heap)):
                    if heap[i][INDICE] == vizinho[0] - 1:
                        heap[i][VALIDO] = False
                        heapq.heappush(
                            heap, [tentativaNovaDistancia, vizinho[0], True])

    # imprimindo vertice destino, caminho percorrido e distancia
    print("\nDijkstra:")
    for i in range(len(distanciaInicial) - 1):
        caminhos = []
        if ancestral[i] is not None:
            caminhos.append(str(i + 1))
            if i + 1 != indiceVertice:
                indice = i
                while True:
                    caminhos.append(str(ancestral[indice]))
                    if ancestral[indice] == indiceVertice:
                        break
                    indice = ancestral[indice] - 1
            caminhos.reverse()

        print(f"{i + 1}: {','.join(caminhos)}; d={distanciaInicial[i]}")
    return distanciaInicial, ancestral


# A partir de um grafo, um vérite inicial e uma lista de aresta visitadas, determina se há um subciclo
def buscarSubciclo(grafo, verticeInicial, arestasVisitadas):
    qtdArestas = grafo.qtdArestas()
    qtdVertices = grafo.qtdVertices()

    tour = []
    verticeAtual = int(verticeInicial)
    while True:
        for u in range(qtdArestas):
            aresta = grafo.arestas()[u]
            if verticeAtual in aresta and arestasVisitadas[u] == False:
                arestasVisitadas[u] = True
                tour.append(verticeAtual)
                verticeSeguinte = [x for x in aresta if x != verticeAtual][0]
                tour.append(verticeSeguinte)
                verticeAtual = verticeSeguinte

        if (verticeInicial == verticeAtual):
            tour = removeDuplicatas(tour)
            return 1, tour, arestasVisitadas

        elif verticeAtual == verticeSeguinte and False not in arestasVisitadas:
            return 0, None, None


# Recebe uma lista de listas e retorna uma lista com todos os elementos
def desempacotar(list_of_lists):
    if len(list_of_lists) == 0:
        return list_of_lists
    if isinstance(list_of_lists[0], list):
        return desempacotar(list_of_lists[0]) + desempacotar(list_of_lists[1:])
    return list_of_lists[:1] + desempacotar(list_of_lists[1:])


# Recebe um grafo e retorna se ele tem um ciclo euleriano (output: 1\n vértices que pertencem a este ciclo) ou não (output: 0)
def buscarSubcicloEuleriano(grafo):
    qtdArestas = grafo.qtdArestas()
    # seleciona um dos vértices
    verticeInicial = 1

    # inicializa a lista de arestasVisitadas
    arestasVisitadas = qtdArestas * [False]

    # chama método buscarSubciclo identificando se há(possuiSubciclo), qual o tour percorrido(tour) e uma lista de arestasVisitadas já modificada
    possuiSubciclo, tour, arestasVisitadasModificadas = buscarSubciclo(
        grafo, verticeInicial, arestasVisitadas)
    if possuiSubciclo == 1:
        arestas = grafo.arestas()

        # enquanto tivermos arestas que não foram visitadas, percorre
        while True:
            if False in arestasVisitadasModificadas:
                indexFalse = arestasVisitadasModificadas.index(False)
                arestaSubciclo = arestas[indexFalse]
                verticeSubciclo = arestaSubciclo[0]

                _, subtour, arestasVisitadas = buscarSubciclo(
                    grafo, verticeSubciclo, arestasVisitadas)

                # Torna a lista de listas(tour) uma lista só
                tour = desempacotar(tour)

                indexTourReplace = tour.index(verticeSubciclo)
                tourAtualizado = tour
                tour.insert(indexTourReplace, subtour)
                if None in tour:
                    # configura output no formato correto
                    print(0)
                    return 0
                elif False not in arestasVisitadas:
                    # configura output no formato correto
                    tourAtualizadoDesempacotado = desempacotar(tourAtualizado)
                    tourAtualizadoDesempacotado.append(
                        tourAtualizadoDesempacotado[0])
                    tourFormatado = ' '.join(
                        map(str, tourAtualizadoDesempacotado))
                    print(1)
                    print(tourFormatado)
                    return 1, tourFormatado


def floyd_warshall(grafo):
    matriz = grafo.matriz()
    qtd_vertices = grafo.qtdVertices()
    for k in range(qtd_vertices):  # k é o vértice intermediário a ser testado
        for i in range(qtd_vertices):
            for j in range(qtd_vertices):
                # se aresta i-> k ou k-> j for infinito a matriz não muda
                if matriz[i][k] != sys.maxsize and matriz[k][j] != sys.maxsize:
                    # se o caminho passando por k for o menor, substitui
                    if matriz[i][k] + matriz[k][j] < matriz[i][j]:
                        # i-> k-> j é menor que i-> j
                        matriz[i][j] = matriz[i][k] + matriz[k][j]

    print_floyd_warshall(matriz)


def print_floyd_warshall(matriz):
    print("\nFloyd Warshall:")
    for sainte in range(len(matriz[0])):
        print(sainte + 1, end="")  # print do vértice
        print(":", end="")
        for entrante in range(len(matriz[sainte])):
            # print das distâncias do vértice sainte pra cada um do outros
            print(int(matriz[sainte][entrante]), end="")
            if (entrante < len(matriz[sainte]) - 1):
                print(",", end="")
        print()


def a1():
    print("Carregando teste com ciclo euleriano")
    grafoComCiclo = Grafo("./tests/ciclo_euleriano/ContemCicloEuleriano.net")
    buscarSubcicloEuleriano(grafoComCiclo)
    print("Carregando teste sem ciclo euleriano")
    grafoSemCiclo = Grafo("./tests/ciclo_euleriano/SemCicloEuleriano.net")
    buscarSubcicloEuleriano(grafoSemCiclo)

    print("Carregando teste dolphins")
    grafo = Grafo("./tests/pequenas/dolphins.net")
    representacao(grafo)
    vertice_buscas = int(input("Entre com vértice para busca em largura: "))
    buscas(grafo, vertice_buscas)

    vertice_dijkstra = int(input("Entre com vértice para dijkstra: "))
    dijkstra(grafo, vertice_dijkstra)
    floyd_warshall(grafo)