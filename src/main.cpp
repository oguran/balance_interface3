/*
 * Copyright (C) 2019 SEG
 */

#include <stdio.h>
#include "balance_interface/balance_interface.h"

const static char VERSION[] = "0.0.002";

int
main(int argc, char **argv)
{
    printf("--- BalanceIF ---V. %s\n", VERSION);
    ros::init(argc, argv, "BalanceIF");

    BalanceIF bl_if;

    bl_if.Init();
    bl_if.Start();

    ros::spin();

    return 0;
}
