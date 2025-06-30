package org.firstinspires.ftc.teamcode.services.Service.Planner;

public interface FieldConfig {
    // Like scoring zones and fixed obstacles, like the 2024 season's "submersible"
    FieldNode[] getFixedFieldNodes();

    // Like initial scoring element types
    FieldNode[] getStartingDynamicFieldNodes();
}
