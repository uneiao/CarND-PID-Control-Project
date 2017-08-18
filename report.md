### REFLECTION

#### PID components

P component is proportional to the cross track error as a feedback to the system.

I component is proportional to the integral sum of cte, in case of
accumulated bias leading to large long term bias.

D component is proportional to the derivative of cte help to avoid overshoot
from P component reacting to the system.


#### Hyperparameters

The hyperparameters are tuned manually.
