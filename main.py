# grafo não-dirigido e ponderado G(V, E, w)
# V é o conjunto de vértices
# E é o conjunto de arestas
# w : E → R é a função que mapeia o peso de cada aresta {u, v} ∈ E
class Grafo:
    def __init__(self, nomeArquivo):
        self.vertices = {}
        self.arestas = []
        self.lerArquivo(nomeArquivo)

    # retorna a quantidade de vértices;
    def qtdVertices():
        pass

    # retorna a quantidade de arestas
    def qtdArestas():
        pass

    # retorna o grau do vértice v
    def grau(v):
        pass

    # retorna o rótulo do vértice v
    def rotulo(v):
        pass

    # retorna os vizinhos do vértice v
    def vizinhos(v):
        pass

    # se {u, v} ∈ E, retorna verdadeiro; se não existir, retorna falso
    def haAresta(v):
        pass

    # se {u, v} ∈ E, retorna o peso da aresta {u, v}; se não existir, retorna um valor infinito positivo 1
    def peso(u,v):
        pass

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
    def lerArquivo(nomeArquivo):
        pass

def main():
    print("printing main")

if __name__ == '__main__':
    main()
