# Regra 4. E:\Projetos\Claude nunca é alterado; shared/ é replicável, o resto só com consentimento

`E:\Projetos\Claude` é um projeto paralelo: leia, mas não altere uma única linha.

- `shared\` pode ser lido e replicado para `E:\Projetos\lidar` como for conveniente (hoje em `lidar\shared`).
- `third-party\`: TODA e qualquer ferramenta está liberada (usuário, 2026-09-26: "toda e qualquer ferramenta
  em third-party está à sua disposição"; reforçado: "uso consentido sem confirmação"), só para
  leitura/execução, sem pedir confirmação; saídas sempre em `lidar\build`.
- As demais pastas (`abCamera\`, `memory\`...) só com consentimento explícito do usuário, pedido a cada uso.
- Nenhuma memória direta daquele projeto é trazida para cá; só a espinha dorsal (regraOuro, arquitetura,
  jobLog).

Origem (usuário, 2026-09-26): "E:\Projetos\Claude é um projeto paralelo totalmente ReadOnly... leia, mas não
altere uma única linha"; "o código em shared é de leitura e pode ser replicado... as demais pastas só devem ser
utilizadas com meu consentimento explícito".
