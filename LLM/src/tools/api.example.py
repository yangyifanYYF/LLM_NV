from openai import OpenAI
    
api_key = "sk-xgB6g47glMCMAQgNC89802454cCb458e9b0c958328525464"
api_base = "https://pro.aiskt.com/v1"
client = OpenAI(api_key=api_key, base_url=api_base)
    
completion = client.chat.completions.create(
    model="gpt-4",
    messages=[
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Hello!"}
    ]
)
    
print(completion.choices[0].message)