# skripsi_ws
The main part of this architecture is divided by two part, high-level architecture and low-level architecture.

### High-Level
This part is responsible to handle all the motion of the humanoid. The resulting data are transmitted to Low-Level.

### Low-Level
All kinds of interaction between software and hardware are handled by this part,
such as receive data from sensor and transmit command to servo .

### Reference
Gait Implementation
> Marcell Missura and Sven Behnke. *Self-stable Omnidirectional Walking with
Compliant Joints*. In 13th IEEE-RAS International Conference on Humanoid Robots (Humanoids), Atalanta, GA, 2013.

### Todos
  - Finishing Low-Level architecture
  - Solve the OpenGL simulation bug
