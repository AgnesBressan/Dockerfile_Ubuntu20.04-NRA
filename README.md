# Workspace NRA - Docker Setup

Este repositório contém um Dockerfile para configurar um ambiente de desenvolvimento completo para o Workspace do NRA (Navegação Robótica Autônoma). A imagem gerada inclui todas as dependências e instâncias necessárias para iniciar e trabalhar no ambiente de desenvolvimento do NRA.

## Requisitos

- [Docker](https://docs.docker.com/get-docker/) instalado em sua máquina.
- Conexão estável com a internet para baixar as dependências.

## Passos para Configuração

1. Clone este repositório:

    ```bash
    git clone https://github.com/seu_usuario/workspace-nra.git
    cd workspace-nra
    ```

2. Build da Imagem Docker:

    ```bash
    docker build -t nra-workspace .
    ```

3. Rodando o Container:

    ```bash
    docker run -it --name nra-container nra-workspace
    ```

4. Persistindo Dados (Usando Volumes):

    ```bash
    docker run -it --name nra-container -v /caminho/para/seu/workspace:/home/nra/workspace nra-workspace
    ```

5. Parando e Reiniciando o Container:

    Para parar o container:
    
    ```bash
    docker stop nra-container
    ```

    Para iniciar novamente:

    ```bash
    docker start -i nra-container
    ```

6. Remover o Container:

    Se você quiser remover o container criado:

    ```bash
    docker rm nra-container
    ```

## Autor

**Agnes Bressan de Almeida**
