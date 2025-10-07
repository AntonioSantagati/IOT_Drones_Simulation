apiVersion: v1
kind: ConfigMap
metadata:
  name: am-logic
  namespace: robotkube
data:
  am_server.py: |
    #!/usr/bin/env python3
    import os, yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, CancelResponse
    from application_manager_interfaces.action import DeploymentRequest
    from kubernetes import client, config
    NS = os.environ.get("AM_NAMESPACE","robotkube")
    APPS_DIR = os.environ.get("AM_APPS_DIR","/apps")
    APP_MAP = {k:v for k,v in (p.split("=",1) for p in os.environ.get("AM_APP_MAP","").split(",") if "=" in p)}
    ACTION_NAME = os.environ.get("AM_ACTION_NAME","/deployment_request")
    class AMServer(Node):
        def __init__(self):
            super().__init__("application_manager")
            config.load_incluster_config()
            self.apps = client.AppsV1Api()
            self.srv = ActionServer(self, DeploymentRequest, ACTION_NAME,
              execute_callback=self.execute_cb,
              cancel_callback=lambda gh: CancelResponse.REJECT)
            self.get_logger().info(f"AM up ns={NS} action={ACTION_NAME} map={APP_MAP}")
        def _apply_if_absent(self, manifest_file: str):
            path = os.path.join(APPS_DIR, manifest_file)
            with open(path,"r") as f: manifest = yaml.safe_load(f)
            name = manifest["metadata"]["name"]
            try:
              self.apps.read_namespaced_deployment(name, NS); exists=True
            except client.exceptions.ApiException as e:
              exists = False if e.status==404 else (_ for _ in ()).throw(e)
            if not exists:
              self.get_logger().info(f"create {name}")
              self.apps.create_namespaced_deployment(NS, manifest)
            return name
        def _scale(self, name: str, replicas: int):
            self.get_logger().info(f"scale {name}={replicas}")
            self.apps.patch_namespaced_deployment(name, NS, {"spec":{"replicas":replicas}})
        async def execute_cb(self, goal_handle):
            goal = goal_handle.request
            ids=[]
            if getattr(goal,"apps",None):
              for app in goal.apps:
                hdr = getattr(app,"header",None)
                app_id = getattr(hdr,"id",None) or getattr(app,"id",None)
                if app_id: ids.append(app_id)
            if not ids and hasattr(goal,"application_id"): ids=[goal.application_id]
            if not ids:
              self.get_logger().warn("No app in goal"); goal_handle.abort()
              return DeploymentRequest.Result()
            for app_id in ids:
              mf = APP_MAP.get(app_id)
              if not mf: self.get_logger().error(f"{app_id} not mapped"); continue
              try:
                name = self._apply_if_absent(mf)
                self._scale(name, 0 if goal.shutdown else 1)
              except Exception as e:
                self.get_logger().error(f"Error {app_id}: {e}")
            goal_handle.succeed(); return DeploymentRequest.Result()
    def main():
      rclpy.init(); node=AMServer()
      try: rclpy.spin(node)
      finally: node.destroy_node(); rclpy.shutdown()
    if __name__=="__main__": main()
