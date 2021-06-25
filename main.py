import sys
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

    # retorna a quantidade de vértices;
    def qtdVertices(self):
        return self.__qtd_vertices

    # retorna a quantidade de arestas
    def qtdArestas(self):
        return self.__qtd_arestas

    # retorna o grau do vértice v
    def grau(self, v):
        return len(self.__vetor[v-1])

    # retorna o rótulo do vértice v
    def rotulo(self, v):
        return self.__rotulos[v-1]

    # retorna lista de tuplas com os vizinhos do vértice v
    def vizinhos(self, v):
        return self.__vetor[v-1]

    # se {u, v} ∈ E, retorna verdadeiro; se não existir, retorna falso
    def haAresta(self, u, v):
        return self.__matriz[u-1][v-1] != self.__infinito

    # se {u, v} ∈ E, retorna o peso da aresta {u, v}; se não existir, retorna um valor infinito positivo 1
    def peso(self, u,v):
        return self.__matriz[u-1][v-1]

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
            self.__matriz = [[self.__infinito] * qtdVertices for _ in range(qtdVertices)]
            arquivo.readline() # pula linha que contem *edges

            arestas = arquivo.readlines()
            self.__qtd_arestas = len(arestas)
            for aresta in arestas:
                v1, v2, peso = aresta.split()
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

    v1, v2 = input("Informe vértices para verificar existencia de aresta e peso: ").split()
    v1 = int(v1)
    v2 = int(v2)
    print(f"Há aresta? {grafo.haAresta(v1, v2)}")
    print(f"Peso: {grafo.peso(v1, v2)}")


def main():
    print("printing main")
    grafo = Grafo("./tests/test0.net")
    representacao(grafo)

if __name__ == '__main__':
    main()
