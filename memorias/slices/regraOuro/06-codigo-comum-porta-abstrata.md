# Regra 6. Código sempre comum às plataformas - adaptadores finos atrás de uma porta abstrata

Não existe código de app específico de plataforma. Um único app comum é escrito contra uma porta abstrata
(métodos descem, eventos sobem por um sink); cada plataforma implementa só o adaptador fino, sem lógica de
captura nem de UI. Layout é decidido apenas no código comum. Precisou de algo novo da plataforma? Novo
método/evento na porta, nunca lógica no adaptador.

Origem (usuário, 2026-09-26): "não temos nenhum código que roda específico em nenhuma plataforma, o código
sempre é comum aos dois e temos as devidas abstrações para que funcione transparente em ambos".

Refinamento (usuário, 2026-09-26): "a leitura de sensores e exibição de componentes fica embutida por
plataforma, como esperado, mas nenhuma regra de negócio da aplicação deve conviver nessa camada". O adaptador
LÊ sensores (câmera, atitude, GPS) e EXIBE componentes (superfície, texto) e entrega fatos crus à porta; toda
decisão (quais frames guardar, conversão de orientação, cobertura do giro, JPEG, sessão, cômodos, dicas ao
operador) vive no app comum.
