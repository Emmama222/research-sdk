import logging

from research_sdk.process_workers.worker import BaseWorker


class _RunningEvent:
    def is_set(self) -> bool:
        return True


class _FailingWorker(BaseWorker):
    def step(self) -> None:
        raise RuntimeError("expected test failure")

    def shutdown(self) -> None:
        self.logger.info("test shutdown")


def test_worker_uses_named_standard_logger_by_default() -> None:
    worker = BaseWorker(_RunningEvent())

    assert isinstance(worker.logger, logging.Logger)
    assert worker.logger.name == "research_sdk.worker.BaseWorker"


def test_worker_uses_supplied_standard_logger() -> None:
    logger = logging.getLogger("tests.supplied-worker")

    worker = BaseWorker(_RunningEvent(), logger)

    assert worker.logger is logger


def test_repeated_worker_errors_are_logged_and_stop_the_loop(caplog) -> None:
    logger = logging.getLogger("tests.failing-worker")
    worker = _FailingWorker(_RunningEvent(), logger)

    with caplog.at_level(logging.ERROR, logger=logger.name):
        worker.run()

    assert worker.error_cnt == 0
    assert sum("worker step failed" in record.message for record in caplog.records) == 4
    assert any("stopping after repeated" in record.message for record in caplog.records)
