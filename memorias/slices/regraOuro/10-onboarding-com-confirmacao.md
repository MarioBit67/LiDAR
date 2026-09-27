# Regra 10. Todo agente novo lê as memórias e confirma verbalmente antes de agir

Cada agente iniciado (sessão principal ou sub-agente) lê, antes de qualquer ação, `memorias/regraOuro.md` (e
as fatias relevantes em `slices/regraOuro/`), `memorias/Arquitetura.md` e `memorias/jobLog.md`, e confirma
verbalmente a leitura de cada um (ex.: "Li regraOuro, arquitetura e jobLog"). O prompt de um sub-agente deve
exigir essa leitura e essa confirmação.

Origem (usuário, 2026-09-26): "MEMORY.md deverá solicitar a leitura desses arquivos com confirmação verbal
para cada novo agente iniciado".
