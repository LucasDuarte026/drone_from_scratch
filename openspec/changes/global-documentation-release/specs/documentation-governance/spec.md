# Specification: Documentation Architecture & Open-Source Governance

## ADDED Requirements

### Requirement: Tri-Tier README Documentation Hierarchy
The repository SHALL maintain three interconnected README documentation artifacts:
1. Root `README.md`: Central entry point defining project identity, academic course metadata, comparison matrix between Project 1 and Project 2, multimedia demonstrations, and repository structure.
2. `project_1/README.md`: Subsystem specification for the physical quadcopter test bench, dual-Arduino wireless RF link, stepper motor control, and `example_01.ino` bug remediation.
3. `project_2/README.md`: Subsystem specification for the PlatformIO flight controller firmware, 250 Hz control loop, automated Pytest suite, and inertial navigation odometry research.

#### Scenario: Navigating to Subsystem Documentation
- **GIVEN** a user or evaluator accessing the repository root `README.md`
- **WHEN** they inspect the subsystem navigation section
- **THEN** direct markdown relative links SHALL guide them directly to `project_1/README.md` and `project_2/README.md` without broken references or `file://` URIs.

---

### Requirement: Academic Attribution & Course Governance
The root documentation SHALL explicitly state the academic context of the project:
1. Course code and name: `SAA0356 - Sistemas Embarcados para Veículos Aéreos (Turma 201)`.
2. Institution: `Escola de Engenharia de São Carlos, Universidade de São Paulo (EESC - USP)`.
3. Faculty advisor: `Prof. Dr. Glauco Augusto de Paula Caurin`, containing verified hyperlinks to his official USP faculty profile and CNPq Lattes curriculum.
4. Team members: David Santiago Correa González, Juan Pablo Solano Mesa, Diego Alonso, and Lucas Duarte.

#### Scenario: Verifying Academic Provenance
- **GIVEN** an academic reviewer evaluating course compliance
- **WHEN** reading the header of `README.md`
- **THEN** the reviewer SHALL find verified clickable links to Prof. Glauco Caurin's USP profile and Lattes curriculum alongside complete student identification.

---

### Requirement: Moderate, Objective Technical Tone
All documentation, architectural diagrams, and code comments SHALL use moderate, factual, and strictly objective engineering prose:
1. Claims MUST be supported by code implementation, formulas, or test evidence.
2. Subjective superlatives, promotional adjectives, and exaggerated self-assessments SHALL NOT be used.
3. Diagrams SHALL use standard Mermaid interpreter styling without custom `style` or `classDef` overrides.

#### Scenario: Auditing Technical Claims
- **GIVEN** a reader reviewing the system description
- **WHEN** reading technical specifications or performance summaries
- **THEN** all descriptions SHALL use straightforward past/present action verbs and cite concrete mechanisms (e.g., "executes in 4000 microseconds", "integrates acceleration twice", "clamps velocity below 0.05g").

---

### Requirement: Ephemeral Lifecycle of OpenSpec Artifacts
The system specification SHALL explicitly mandate that all `openspec/` folders (`openspec/`, `project_1/openspec/`, and `project_2/openspec/`) represent internal development governance tools:
1. All permanent architectural documentation, register maps, and operational guides SHALL reside in `docs/` directories and README files.
2. The final public release checklist SHALL mandate omitting or deleting `openspec/` directories prior to public distribution.

#### Scenario: Preparing Final Open-Source Release
- **GIVEN** the team preparing the repository for final public release
- **WHEN** executing the release packaging checklist
- **THEN** internal `openspec/` directories SHALL be purged, leaving `docs/` and README files as the sole living documentation.
