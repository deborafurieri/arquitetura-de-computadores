# Laboratório para Simulação Prática de Testbench

Esse laboratório tem por objetivo a realização de uma prática de avaliação e simulação de código em processador ARM de prova de conceito, disponibilizado nas versões monociclio, multiciclo e pipelined, conforme apresentado em sala de aula.

## Análise manual

Considerando o código Assembly em memfile.s e o código disponível em memfile.dat, verifique a reciprocidade dos códigos. Considerando a fórmula de avaliação de tempo de execução, para avaliação hipotética manual, faça uma estimativa do tempo de execução esperado para cada processador. Descubra, por meio do módulo de testbench inerente a cada simulador, o tempo de clock, verifique o número de instruções e o CPI de cada processador.

### Execução do Código no Processador em Ambiente Simulado

Utilizando o VSIM, inicialize um projeto com os respectivos processadores. Utilizando-se da prática com simulação da ULA, execute a simulação e verifique as instruções em execução na timeline de simulação. Identifique as instruções correspondentes no arquivo memfile.dat.

Compare a execução das instruções nos processadores monociclo, multiciclo e pipeline, identifique os módulos chaves da simulação do datapath e da unidade de controle e analise o passado a passo da execução da instrução.

Verifique o tempo de execução do programa em cada processador utilizando a timeline e a saída do testbench. Analisando o testbench, qual é a verificação automática utilizada no testbench para validação no terminal?

## Resultados
Envie os resultados obtidos em um relatório simples da sua dupla no respectivo espaço disponível no AVA. Utilize um projeto GIT para esse fim, se preferir, e compartilhe o acesso junto ao professor.
