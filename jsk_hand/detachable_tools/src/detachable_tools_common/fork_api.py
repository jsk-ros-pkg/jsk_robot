"""ROS 2 client for the shared Nextage right-arm fork exchange server."""
import json
from std_srvs.srv import Trigger


class NextageForkAPI:
    """Use from a node spun by another executor/thread, or await the returned future.

    plan_detach/plan_attach preview; detach/attach execute only if the server was
    started with --hardware and a measured, calibrated configuration.
    """
    def __init__(self, node, namespace='/fork_exchange'):
        self.node = node
        self.clients = {name: node.create_client(Trigger, namespace.rstrip('/') + '/' + name)
                        for name in ('plan_detach', 'plan_attach', 'detach', 'attach')}

    def call_async(self, operation, timeout=3.0):
        client = self.clients[operation]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'Fork service unavailable: {operation}')
        return client.call_async(Trigger.Request())

    @staticmethod
    def result(response):
        if not response.success:
            raise RuntimeError(response.message)
        return json.loads(response.message)

    def plan_detach(self):
        return self.call_async('plan_detach')

    def plan_attach(self):
        return self.call_async('plan_attach')

    def detach(self):
        return self.call_async('detach')

    def attach(self):
        return self.call_async('attach')
