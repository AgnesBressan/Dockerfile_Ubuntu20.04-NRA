# Workspace NRA - Docker Setup

Este repositório contém um Dockerfile para configurar um ambiente de desenvolvimento completo para o Workspace do NRA (Núcleo Robótica Aérea). A imagem gerada inclui todas as dependências e instâncias necessárias para iniciar e trabalhar no ambiente de desenvolvimento do NRA.

## Requisitos

- [Docker](https://docs.docker.com/get-docker/) instalado em sua máquina.
- Conexão estável com a internet para baixar as dependências.
- Acesso ao repositório privado `cbr_2024` com um token de acesso pessoal (PAT) do GitHub.
   __(Opcional para caso queira utilizar o repositório cbr_2024).__

## Passos para Configuração

1. **Clone este repositório:**

    ```bash
    git clone https://github.com/seu_usuario/workspace-nra.git
    cd workspace-nra
    ```

2. **Tornar os scripts executáveis:**

    Antes de buildar e rodar a imagem, torne os scripts `build.sh` e `run.sh` executáveis:

    ```bash
    chmod +x build.sh run.sh
    ```

3. **Build da Imagem Docker:**

    Para buildar a imagem Docker, execute o script `build.sh`:

    ```bash
    ./build.sh
    ```

4. **Rodando o Container:**

    Após o build da imagem, execute o script `run.sh` para rodar a imagem e criar o container:

    ```bash
    ./run.sh
    ```

5. **Parando e Reiniciando o Container:**

    Para parar o container:

    ```bash
    docker stop nra-container
    ```

    Para iniciar novamente:

    ```bash
    docker start -i nra-container
    ```

6. **Remover o Container:**

    Se você quiser remover o container criado:

    ```bash
    docker rm nra-container
    ```

---

## Integração com o Repositório da CBR 2024

Para adicionar o repositório da CBR 2024 ao seu ambiente Docker, siga os passos abaixo. Certifique-se de substituir as informações de configuração do Git por seus próprios dados antes de construir a imagem.

### Instruções para Configuração no Dockerfile

Adicione as seguintes linhas ao seu Dockerfile para configurar os modelos e scripts necessários da CBR 2024:

1. Configura o Git (Substitua com suas informações):

    ```dockerfile
    RUN git config --global user.email "seu_email@exemplo.com"
    RUN git config --global user.name "SeuNome"

2. Configurar o Dockerfile com o repositório privado da CBR 2024:

    Para adicionar o repositório privado `cbr_2024` ao ambiente Docker, inclua a linha abaixo no Dockerfile. Essa linha fará o clone do repositório `cbr_2024`:

    ```dockerfile
    RUN cd ~/catkin_ws/src \
        && git clone https://<SEU_TOKEN>@github.com/Grupo-SEMEAR-USP/cbr_2024.git ~/catkin_ws/src/cbr_2024
    ```

    **Substitua `<SEU_TOKEN>` pelo seu token de acesso pessoal (PAT)**

    #### Como Criar o Token de Acesso Pessoal (PAT)
    1. Acesse [GitHub Settings](https://github.com/settings/tokens).
    2. Clique em **"Generate new token"** e selecione as permissões necessárias para acessar o repositório privado.
    3. Copie o token gerado e substitua `<SEU_TOKEN>` na linha acima pelo seu token.

3. Cria o diretório .gazebo/models e copia os modelos da CBR 2024 para esse diretório:

    ```dockerfile
    RUN mkdir -p /root/.gazebo/models \
        && cp -r ~/catkin_ws/src/cbr_2024/cbr/models/* /root/.gazebo/models/

4. Executa o script de setup da CBR 2024:

    ```dockerfile
    RUN cd /root/catkin_ws/src/cbr_2024/setup \
        && ls -la \
        && bash setup.sh \
        && cd /root/catkin_ws

## Autor

**[Agnes Bressan de Almeida](https://github.com/AgnesBressan)**  
**[Marco Tayar](https://github.com/MarcoTayar)**
