FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    TZ=Asia/Shanghai

WORKDIR /app

RUN addgroup --system decision && adduser --system --ingroup decision decision
COPY --chown=decision:decision server/decision_service.py server/jev_policy.py server/control_runtime.py /app/server/
COPY --chown=decision:decision server/policies /app/server/policies/
COPY --chown=decision:decision server/navigation /app/server/navigation/
COPY --chown=decision:decision jev_api/choice_control.py jev_api/route_context.py /app/jev_api/
COPY --chown=decision:decision jev_api/tools /app/jev_api/tools/

USER decision
ENTRYPOINT ["python", "/app/server/decision_service.py"]
