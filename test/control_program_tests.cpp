#include <gtest/gtest.h>

#include "../include/control/control_program.h"
#include "../include/powertrain/scripted_control_unit.h"
#include "../include/units.h"

#include <cmath>
#include <string>

namespace {
    control::ConstantBlock *constant(control::ControlProgram *program, double value) {
        control::ConstantBlock *block = new control::ConstantBlock;
        block->m_value = value;
        program->addBlock(block);

        return block;
    }

    template <typename T>
    T *add(control::ControlProgram *program, const char *name = "") {
        T *block = new T;
        block->m_name = name;
        program->addBlock(block);

        return block;
    }
}

TEST(ControlProgramTests, ABlockChainEvaluatesInDependencyOrder) {
    control::ControlProgram program;

    control::ConstantBlock *a = constant(&program, 3.0);
    control::ConstantBlock *b = constant(&program, 4.0);

    control::SumBlock *sum = add<control::SumBlock>(&program, "sum");
    sum->addOperand(a->m_index);
    sum->addOperand(b->m_index);

    control::GainBlock *gain = add<control::GainBlock>(&program, "gain");
    gain->m_gain = 2.0;
    gain->m_offset = 1.0;
    gain->addOperand(sum->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();
    program.update(1e-3);

    EXPECT_NEAR(sum->m_output, 7.0, 1e-12);
    EXPECT_NEAR(gain->m_output, 15.0, 1e-12);
}

TEST(ControlProgramTests, DeclarationOrderDoesNotMatter) {
    control::ControlProgram program;

    control::GainBlock *gain = add<control::GainBlock>(&program, "gain");
    gain->m_gain = 10.0;

    control::SumBlock *sum = add<control::SumBlock>(&program, "sum");
    control::ConstantBlock *a = constant(&program, 1.5);

    gain->addOperand(sum->m_index);
    sum->addOperand(a->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();
    program.update(1e-3);

    EXPECT_NEAR(gain->m_output, 15.0, 1e-12);
}

TEST(ControlProgramTests, ACycleWithoutADelayIsRejected) {
    control::ControlProgram program;

    control::SumBlock *a = add<control::SumBlock>(&program, "a");
    control::SumBlock *b = add<control::SumBlock>(&program, "b");

    a->addOperand(b->m_index);
    b->addOperand(a->m_index);

    EXPECT_FALSE(program.compile());
    EXPECT_NE(program.getError().find("cycle"), std::string::npos);
    EXPECT_NE(program.getError().find("delay"), std::string::npos);
}

TEST(ControlProgramTests, ADelayBlockBreaksTheCycleAndHoldsOneTick) {
    control::ControlProgram program;

    control::ConstantBlock *one = constant(&program, 1.0);

    control::SumBlock *accumulator = add<control::SumBlock>(&program, "accumulator");
    control::DelayBlock *delay = add<control::DelayBlock>(&program, "delay");

    accumulator->addOperand(one->m_index);
    accumulator->addOperand(delay->m_index);
    delay->addOperand(accumulator->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    for (int i = 1; i <= 5; ++i) {
        program.update(1e-3);
        EXPECT_NEAR(accumulator->m_output, static_cast<double>(i), 1e-12) << "tick " << i;
    }
}

TEST(ControlProgramTests, TheIntegratorFollowsTheAnalyticSolution) {
    control::ControlProgram program;

    control::ConstantBlock *rate = constant(&program, 2.0);

    control::IntegratorBlock *integrator =
        add<control::IntegratorBlock>(&program, "integrator");
    integrator->m_min = -1e9;
    integrator->m_max = 1e9;
    integrator->addOperand(rate->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    const double dt = 1e-3;
    for (int i = 0; i < 1000; ++i) program.update(dt);

    EXPECT_NEAR(integrator->m_output, 2.0, 1e-9);
}

TEST(ControlProgramTests, TheIntegratorRespectsItsLimits) {
    control::ControlProgram program;

    control::ConstantBlock *rate = constant(&program, 100.0);

    control::IntegratorBlock *integrator =
        add<control::IntegratorBlock>(&program, "integrator");
    integrator->m_min = 0.0;
    integrator->m_max = 1.0;
    integrator->addOperand(rate->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    for (int i = 0; i < 1000; ++i) program.update(1e-3);

    EXPECT_NEAR(integrator->m_output, 1.0, 1e-12);
}

TEST(ControlProgramTests, TheLowPassBlockMatchesTheFirstOrderStepResponse) {
    control::ControlProgram program;

    control::ConstantBlock *step = constant(&program, 0.0);

    control::LowPassBlock *filter = add<control::LowPassBlock>(&program, "filter");
    filter->m_timeConstant = 0.1;
    filter->addOperand(step->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    const double dt = 1e-4;
    program.update(dt);
    ASSERT_NEAR(filter->m_output, 0.0, 1e-12);

    step->m_value = 1.0;

    const int steps = static_cast<int>(0.1 / dt);
    for (int i = 0; i < steps; ++i) program.update(dt);

    EXPECT_NEAR(filter->m_output, 1.0 - std::exp(-1.0), 1e-3);
}

TEST(ControlProgramTests, TheMinBlockActsAsATorqueCoordinator) {
    control::ControlProgram program;

    control::ConstantBlock *driver = constant(&program, 180.0);
    control::ConstantBlock *limiter = constant(&program, 90.0);
    control::ConstantBlock *traction = constant(&program, 140.0);

    control::MinBlock *coordinator = add<control::MinBlock>(&program, "coordinator");
    coordinator->addOperand(driver->m_index);
    coordinator->addOperand(limiter->m_index);
    coordinator->addOperand(traction->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();
    program.update(1e-3);

    EXPECT_NEAR(coordinator->m_output, 90.0, 1e-12);
}

TEST(ControlProgramTests, TheCompareBlockHoldsItsStateInsideTheBand) {
    control::ControlProgram program;

    control::ConstantBlock *measurement = constant(&program, 0.0);
    control::ConstantBlock *threshold = constant(&program, 0.0);

    control::CompareBlock *compare = add<control::CompareBlock>(&program, "compare");
    compare->m_band = 1.0;
    compare->addOperand(measurement->m_index);
    compare->addOperand(threshold->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    measurement->m_value = 0.5;
    program.update(1e-3);
    EXPECT_NEAR(compare->m_output, 0.0, 1e-12);

    measurement->m_value = 1.5;
    program.update(1e-3);
    EXPECT_NEAR(compare->m_output, 1.0, 1e-12);

    measurement->m_value = 0.5;
    program.update(1e-3);
    EXPECT_NEAR(compare->m_output, 1.0, 1e-12);

    measurement->m_value = -1.5;
    program.update(1e-3);
    EXPECT_NEAR(compare->m_output, 0.0, 1e-12);
}

TEST(ControlProgramTests, TheLatchRemembersUntilItIsReset) {
    control::ControlProgram program;

    control::ConstantBlock *set = constant(&program, 0.0);
    control::ConstantBlock *reset = constant(&program, 0.0);

    control::LatchBlock *latch = add<control::LatchBlock>(&program, "latch");
    latch->addOperand(set->m_index);
    latch->addOperand(reset->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    program.update(1e-3);
    EXPECT_NEAR(latch->m_output, 0.0, 1e-12);

    set->m_value = 1.0;
    program.update(1e-3);
    EXPECT_NEAR(latch->m_output, 1.0, 1e-12);

    set->m_value = 0.0;
    program.update(1e-3);
    EXPECT_NEAR(latch->m_output, 1.0, 1e-12);

    reset->m_value = 1.0;
    program.update(1e-3);
    EXPECT_NEAR(latch->m_output, 0.0, 1e-12);
}

TEST(ControlProgramTests, TheTimerCountsOnlyWhileTheConditionHolds) {
    control::ControlProgram program;

    control::ConstantBlock *condition = constant(&program, 1.0);

    control::TimerBlock *timer = add<control::TimerBlock>(&program, "timer");
    timer->addOperand(condition->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    for (int i = 0; i < 100; ++i) program.update(1e-3);
    EXPECT_NEAR(timer->m_output, 0.1, 1e-9);

    condition->m_value = 0.0;
    program.update(1e-3);
    EXPECT_NEAR(timer->m_output, 0.0, 1e-12);
}

TEST(ControlProgramTests, ResetClearsEveryStatefulBlock) {
    control::ControlProgram program;

    control::ConstantBlock *rate = constant(&program, 1.0);

    control::IntegratorBlock *integrator =
        add<control::IntegratorBlock>(&program, "integrator");
    integrator->addOperand(rate->m_index);

    control::TimerBlock *timer = add<control::TimerBlock>(&program, "timer");
    timer->addOperand(rate->m_index);

    ASSERT_TRUE(program.compile()) << program.getError();

    for (int i = 0; i < 100; ++i) program.update(1e-3);
    ASSERT_GT(integrator->m_output, 0.0);
    ASSERT_GT(timer->m_output, 0.0);

    program.reset();
    EXPECT_NEAR(integrator->m_output, 0.0, 1e-12);
    EXPECT_NEAR(timer->m_output, 0.0, 1e-12);
}
