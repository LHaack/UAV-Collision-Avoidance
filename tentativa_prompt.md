Antes de iniciar, você deve implementar isso em uma função nova e guardar a "antiga" (que está atualmente implementada) como <nome da função)_OLD_MOST_RECENT

Vamos fazer o seguinte: vamos tentar uma abordagem mais matemática.

Antes de começar efetivamente, vamos definir algumas coisas:

Lidar: eu vou tratar o lidar (aqui entre nós, não necessariamente no código) como uma função L(x) tal que L(x) = 1 quando há obstáculos no ângulo x e L(y) = 0 quando não há obstáculos no ângulo y.

Considere que trataremos de ângulos x tais que -90 < x < 90, onde esse ângulo começa a ser contado da esquerda para a direita (sentido horário) a partir do ângulo mais à esquerda de onde o drone está olhando, e vai até o ângulo mais à direita de onde o drone está olhando.

Basicamente, o campo de visão do drone pode ser definido da seguinte forma: campo = {x e R | -90 < x < 90 } - ou seja, são os 180 graus que o drone pode visualizar com o Lidar.

Agora, vamos definir os tipos de situações que o drone pode encontrar e quais decisões ele deve tomar ao encontrar cada uma dessas situações:

Situação do tipo 1, "campo de visão livre à esquerda", pode ser definida da seguinte maneira: para o campo de visão do drone, considerando x um ângulo da visão do drone, é todo x < y, onde y é o primeiro ângulo no nosso sentido de "varrer" para que L(y) = 1. Ou seja, para todo x tal que -90 < x < y, temos L(x) = 0.

Agora, temos a situação de tipo 2, "campo de visão livre à direita", que pode ser definida da seguinte maneira: Para todo x > y, em que y é o último ângulo tal que L(y) = 1. Ou seja, para todo x tal que -90 < y < x, temos L(x) = 0. Basicamente, todos os lidares de -90 até x indicam que há obstáculos, e tudo depois de x é livre (tudo até 90 graus).

Por último, temos uma situalçao de tipo 3, chama de "buraco", que possui subcasos (podem existir situações de tipo 3 e ao mesmo tempo situações de tipo 1 e 2, assim como mais uma situação de tipo 3 para o campo de visão do drone), que é o seguinte: Temos x, y e z, y < x < z, tais que L(y) = 1, L(x) = 0 e L(z) = 0, onde L(y) é o último y < x tal que L(y) = 1 e z é tal que ele é o primeiro z que satisfaz, para x < z, L(z) = 1.

Portanto, para cada situação, você deve considerar o seguinte:

Numa situação puramente de tipo 1, você deve seguir para a ESQUERDA e parar TODO e qualquer movimento para frente e trás. A velocidade com que o drone vai para a esquerda vai ser dada pela seguinte fórmula: MAX_SPEED * |(|y| - 90)/90|, onde y é o primeiro raio do lidar, varrendo da esquerda para a direita, que indica que há obstáculos. Isso vai servir para fazer com que o drone vá desacelerando conforme chega próximo ao final do obstáculo.

Para a situação de tipo 2, faremos o seguinte: você deve seguir para a DIREITA e parar TODO e qualquer movimento para frente e trás. A velocidade com que o drone vai para a direita vai ser dada pela seguinte fórmula: MAX_SPEED * |(|y| - 90)/90|, onde y é o último raio do lidar, varrendo do raio "perpendicular" ao drone - o bem no meio da "cabeça" do drone - para a direita, que indica que há obstáculos.

Para a situação de tipo 3, faremos o seguinte: se houver apenas um "buraco" livre no campo de visão, você deverá mover o drone lateralmente a fim de deixar esse único "buraco" na frente do drone - o "meio do buraco" deve coincidir com o meio da "testa" do drone. Se hoouver mais de um buraco, você vai dar prioridade sempre para deixar o buraco que estiver mais à DIREITA - ou seja, vai ficar se movendo para a direita - até que o buraco que antes estava à direita esteja com o seu centro bem no centro da testa do drone, como mencionado anteriormente para o outro caso. Então, o drone irá para frente e atravessará o buraco.

Você pode fazer uma máquina de estados para definir quando o drone detecta os buracos e começa a se posicionar e, após estar no meio, ele entra em modo de ir para frente e parar de ver os buracos até que o seu lidar da frente esteja completamente livre (a não ser que apareçam outros obstáculos à frente, nesse caso ele entra em modo de desvio novamente e segue até conseguir uma frente totalmente livre).