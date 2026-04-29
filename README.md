[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/7bjYLsjm)
# Proyecto final

## Objetivo

Desarrollar una aplicación robótica que:

- Navegue de forma autónoma por al menos dos waypoints.
- Interactúe con el humano mediante el paquete HRI (Human-Robot Interaction).
- Utilice YOLO para la detección de objetos o personas.

## Configuración del sistema HRI

Consulta las instrucciones completas en el [repositorio de simple_hri](https://github.com/rodperex/simple_hri#launch-local-services).

Se recomienda usar el **modelo local** para evitar el consumo de tokens externos, para ello:

Lanzar los servicios locales:

```bash
ros2 launch simple_hri local_simple_hri.launch.py
```

Probar los servicios:

```bash
ros2 run simple_hri test_services
```
